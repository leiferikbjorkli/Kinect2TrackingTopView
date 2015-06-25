//
// Written by Leif Erik Bjoerkli
//


namespace InteractionDetection
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Windows;
    using Microsoft.Kinect;

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window
    {
        /// <summary>
        /// Timestamp of previous frame
        /// </summary>
        private TimeSpan lastTime;

        /// <summary>
        /// Points in camera space mapped from depthframe
        /// </summary>
        private CameraSpacePoint[] cameraSpacePoints = null;
        
        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Reader for depth frames
        /// </summary>
        private DepthFrameReader depthFrameReader = null;

        /// <summary>
        /// Description of the data contained in the depth frame
        /// </summary>
        private FrameDescription depthFrameDescription = null;

        /// <summary>
        /// Perform diagnostics on solution
        /// </summary>
        private Diagnostics diagnostics;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        /// 
        public MainWindow()
        {
            
            // get the kinectSensor object
            this.kinectSensor = KinectSensor.GetDefault();

            // open the reader for the depth frames
            this.depthFrameReader = this.kinectSensor.DepthFrameSource.OpenReader();

            // wire handler for frame arrival
            this.depthFrameReader.FrameArrived += this.Reader_FrameArrived;

            // get FrameDescription from DepthFrameSource
            this.depthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // initialize array to store points in camera space of arriving
            // frame
            this.cameraSpacePoints = new CameraSpacePoint[depthFrameDescription.Width * depthFrameDescription.Height];

            // initialize diagnostics object
            diagnostics = new Diagnostics();

            // initialize global coordinate mapper from kinect sensor
            GlobVar.CoordinateMapper = this.kinectSensor.CoordinateMapper;

            // initialize global variable that stores the max amount of detected
            // bodies in session for labeling
            GlobVar.MaxBodyCount = 0;

            // open the sensor
            this.kinectSensor.Open();

            // initialize the components (controls) of the window
            this.InitializeComponent();
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            // Calculate tracking success of session
            diagnostics.CalculateTrackingSuccess();

            if (this.depthFrameReader != null)
            {
                // DepthFrameReader is IDisposable
                this.depthFrameReader.Dispose();
                this.depthFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        /// <summary>
        /// Handles the depth frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, DepthFrameArrivedEventArgs e)
        {
            ushort[] frameData = new ushort[depthFrameDescription.Width*depthFrameDescription.Height];

            using (DepthFrame depthFrame = e.FrameReference.AcquireFrame())
            {
                if (depthFrame != null)
                {
                    Stopwatch stopwatch = new Stopwatch();
                    stopwatch.Start();

                    depthFrame.CopyFrameDataToArray(frameData);
                    depthFrameReader.IsPaused = true;
                    GlobVar.CoordinateMapper.MapDepthFrameToCameraSpace(frameData, this.cameraSpacePoints);

                    PreprocessFrame();

                    AnalyzeFrame(depthFrame.RelativeTime);

                }
           }
        }

        private void PreprocessFrame()
        {
            Stopwatch stopwatch = new Stopwatch();
            stopwatch.Start();

            GlobVar.ScaledCameraSpacePoints = KinectUtils.ScaleFrame(this.cameraSpacePoints);

            GlobVar.PointCloud = KinectUtils.CreatePointCloudPerspective(GlobVar.ScaledCameraSpacePoints);
            if (Diagnostics.LogTime)
            {
                stopwatch.Stop();
                Console.WriteLine("PointCloud: {0}", stopwatch.ElapsedMilliseconds);
                stopwatch.Restart();
            }

            GlobVar.MedianFilteredPointCloud =
            ImageUtils.MedianFilter3X3PointCloud(GlobVar.PointCloud);
            if (Diagnostics.LogTime)
            {
                stopwatch.Stop();
                Console.WriteLine("MedianFilter: {0}", stopwatch.ElapsedMilliseconds);
                stopwatch.Restart();
            }

        }

        private void AnalyzeFrame(TimeSpan frameRelativeTime)
        {
            Stopwatch stopwatch = new Stopwatch();
            stopwatch.Start();

            Point[] candidatesHaar = HaarDetector.CreateRegions(GlobVar.MedianFilteredPointCloud);
            if (Diagnostics.LogTimeHaar)
            {
                stopwatch.Stop();
                Console.WriteLine("Haar: {0}", stopwatch.ElapsedMilliseconds);
                stopwatch.Restart();
            }

            List<Head> validatedCandidateHeads = new List<Head>();
            for (int i = 0; i < candidatesHaar.Length; i++)
            {
                int highestPointIndex = ClassificationUtils.GetHighestPointInSurroundingArea(candidatesHaar[i]);

                List<int> headPointIndexes = ClassificationUtils.ConnectedComponentLabelingIterative(highestPointIndex, 150);

                if (Validators.IsSphericalShape(headPointIndexes, highestPointIndex) && Validators.SizeOfHead(headPointIndexes))
                {
                    var head = new Head(GlobVar.MedianFilteredPointCloud[highestPointIndex]);
                    var success = head.AddHeadPixels(headPointIndexes);
                    if (success == 1)
                    {
                        validatedCandidateHeads.Add(head);
                    }
                }
            }
            GlobVar.ValidatedCandidateHeads = validatedCandidateHeads;

            GlobVar.AdjacancyList = GeodesicUtils.CreateGeodesicGraph(GlobVar.MedianFilteredPointCloud);
            if (Diagnostics.LogTime)
            {
                stopwatch.Stop();
                Console.WriteLine("CreateGeodesicGraph: {0}", stopwatch.ElapsedMilliseconds);
                stopwatch.Restart();
            }

            if (Diagnostics.ShowGeodesicGraph)
            {
                foreach (var v in GlobVar.AdjacancyList)
                {
                    GlobVar.Canvas[v.Key] = 255;
                }
            }

            var bodies = new List<Body>();
            foreach (var candidateHead in validatedCandidateHeads)
            {
                Stopwatch stopwatchBody = new Stopwatch();
                stopwatchBody.Start();

                if (Diagnostics.ShowHeadPixels)
                {
                    foreach (var headPixel in candidateHead.HeadPointIndexes)
                    {
                        GlobVar.Canvas[headPixel] = 255;
                    }
                }

                int bodyId = BodyUtils.GetBodyIdFromHead(candidateHead);

                Body body;

                if (bodyId == -1)
                {
                    body = new Body(GlobVar.MaxBodyCount, frameRelativeTime);
                    body.AddHead(candidateHead);
                    bodies.Add(body);
                    GlobVar.MaxBodyCount++;

                    GeodesicUtils.AddBodyLandmarks(candidateHead, body);
                    if (Diagnostics.LogTime)
                    {
                        stopwatch.Stop();
                        Console.WriteLine("ShortestPath: {0}", stopwatch.ElapsedMilliseconds);
                        stopwatch.Restart();
                    }
                }
                else
                {
                    body = new Body(bodyId, frameRelativeTime);
                    body.AddHead(candidateHead);
                    bodies.Add(body);

                    GeodesicUtils.AddBodyLandmarks(candidateHead, body);
                    if (Diagnostics.LogTime)
                    {
                        stopwatch.Stop();
                        Console.WriteLine("ShortestPath: {0}", stopwatch.ElapsedMilliseconds);
                        stopwatch.Restart();
                    }
                }
                EnergyUtils.UpdateEnergy(body);
                if (Diagnostics.LogTime)
                {
                    stopwatchBody.Stop();
                    Console.WriteLine("ProcessOneBody: {0}", stopwatchBody.ElapsedMilliseconds);
                }
            }

            BodyUtils.UpdateBodyHistory(bodies);
            diagnostics.AddNewBodyFrameDiagnostics(bodies, frameRelativeTime);

            Graphics.RenderDepthPixels(this);

            depthFrameReader.IsPaused = false;

            if (Diagnostics.PrintFPS)
            {
                var currentTime = frameRelativeTime;
                Console.WriteLine("FPS: {0}", GlobUtils.CalculateFPS(currentTime, lastTime));
                lastTime = currentTime;
            }

        }
    }
}
