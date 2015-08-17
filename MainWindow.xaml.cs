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

        private HeatMap heatMap;

        private EnergyUtils energyUtils;

        private StatisticsWindow statWindow;

        private Stopwatch stopwatch;

        private TemporalMedianImage temporalMedianImage;

        private int temporalFrameCounter;

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

            // initialize heatmap analysis
            heatMap = new HeatMap();

            // initialize energy analysis
            energyUtils = new EnergyUtils();

            // initialize global coordinate mapper from kinect sensor
            GlobVar.CoordinateMapper = this.kinectSensor.CoordinateMapper;

            // initialize global variable that stores the max amount of detected
            // bodies in session for labeling
            GlobVar.MaxBodyCount = 0;

            // open the sensor
            this.kinectSensor.Open();

            // initialize the components (controls) of the window
            this.InitializeComponent();

            BodyUtils.InitializeBodyHistory();

            statWindow = new StatisticsWindow();

            stopwatch = new Stopwatch();

            temporalFrameCounter = 30;
            temporalMedianImage = new TemporalMedianImage(temporalFrameCounter);

        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
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

            PostProcessing();
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
                    depthFrameReader.IsPaused = true;
                    depthFrame.CopyFrameDataToArray(frameData);
                    GlobVar.CoordinateMapper.MapDepthFrameToCameraSpace(frameData, this.cameraSpacePoints);

                    var preProcessedFrame = PreProcessFrame(cameraSpacePoints);

                    if (temporalMedianImage.ImageSet)
                    {
                        var subtractedFrame = temporalMedianImage.Subtract(preProcessedFrame);

                        stopwatch.Restart();

                        var filteredFrame = ImageUtils.MedianFilter3X3(subtractedFrame,2);

                        foreach (var cameraSpacePoint in filteredFrame)
                        {
                            if (cameraSpacePoint.Z != GlobVar.MaxDepthMeter && float.IsInfinity(cameraSpacePoint.X) && float.IsInfinity(cameraSpacePoint.Y) && cameraSpacePoint.X == 0 && cameraSpacePoint.Y == 0 || cameraSpacePoint.Z == 0) 
                            {
                                
                            }
                        }

                        if (Diagnostics.LogTimeMedian)
                        {
                            stopwatch.Stop();
                            Console.WriteLine("Median: {0}", stopwatch.ElapsedMilliseconds);
                        }
                        stopwatch.Restart();

                        GlobVar.SubtractedFilteredPointCloud = filteredFrame;

                        AnalyzeFrame(depthFrame.RelativeTime, filteredFrame);

                        GlobVar.PreviousFrame = filteredFrame;
                        GraphicsUtils.RenderDepthPixels(this, filteredFrame);

                    }
                    depthFrameReader.IsPaused = false;
                }
           }
        }

        private CameraSpacePoint[] PreProcessFrame(CameraSpacePoint[] cameraSpacePoints)
        {
            stopwatch.Restart();

            var processedFrame = new CameraSpacePoint[cameraSpacePoints.Length];

            processedFrame = KinectUtils.ScaleFrame(cameraSpacePoints);
            if (Diagnostics.LogTime)
            {
                stopwatch.Stop();
                Console.WriteLine("ScaleFrame: {0}", stopwatch.ElapsedMilliseconds);
            }
            stopwatch.Restart();

            GlobVar.ScaledCameraSpacePoints = processedFrame;

            //processedFrame =
            //KinectUtils.DepthFilterBackgroundSubtraction(GlobVar.ScaledCameraSpacePoints);
            //if (Diagnostics.LogTime) {
            //    stopwatch.Stop();
            //    Console.WriteLine("DepthFilterBackgroundSubtraction: {0}", stopwatch.ElapsedMilliseconds);
            //}
            //stopwatch.Restart();

            if (!temporalMedianImage.ImageSet)
            {
                temporalMedianImage.AddFrame(processedFrame);
            }

            return processedFrame;
        }

        private List<Head> DetectHeads(CameraSpacePoint[] filteredFrame)
        {
            stopwatch.Restart();

            List<int> candidatesHaar = HaarDetector.CreateRegions(filteredFrame);
            if (Diagnostics.LogTimeHaar)
            {
                stopwatch.Stop();
                Console.WriteLine("Haar: {0}", stopwatch.ElapsedMilliseconds);
            }
            stopwatch.Restart();

            List<int> candidateHighestPointIndex = new List<int>();
            for (int i = 0; i < candidatesHaar.Count; i++)
            {
                if (Diagnostics.ShowCandidateHeadpixel)
                {
                    GraphicsUtils.DrawPoint(candidatesHaar[i]);
                }

                int candidateIndex = candidatesHaar[i];

                if (candidateIndex != -1)
                {
                    int highestPointIndex = ClassificationUtils.GetHighestConnectingPoint(candidateIndex, Thresholds.ClassificationSearchDepthHighestPoint);
                    candidateHighestPointIndex.Add(highestPointIndex);

                    if (Diagnostics.ShowTopHeadpixel)
                    {
                        GraphicsUtils.DrawPoint(highestPointIndex);
                    }
                }
            }

            var groupedHighestIndexes = Validators.GroupCandidateHighestPoints(candidateHighestPointIndex, Thresholds.ValidatorsHighestPointGroupingDistance);

            if (groupedHighestIndexes.Count > 1)
            {
                
            }

            List<Head> validatedCandidateHeads = new List<Head>();
            for (int i = 0; i < groupedHighestIndexes.Count; i++)
            {

                var highestPointIndex = groupedHighestIndexes[i];
                if (Diagnostics.ShowTopHeadPixelAfterGrouping)
                {
                    GraphicsUtils.DrawPoint(highestPointIndex);
                }

                List<int> headPointIndexes = ClassificationUtils.ConnectedComponentLabelingIterative(highestPointIndex, Thresholds.ClassificationLabelingMaxPoints);

                if (Diagnostics.ShowHeadpixelsBeforeValidation)
                {
                    foreach (var p in headPointIndexes)
                    {
                        GraphicsUtils.DrawPoint(p);
                    }
                }

                if (Validators.SizeOfHead(headPointIndexes) && Validators.MovementInFrame(headPointIndexes) && Validators.IsSphericalShape(headPointIndexes, highestPointIndex))
                {
                    if (Diagnostics.ShowValidatedTopHeadpixel)
                    {
                        GraphicsUtils.DrawPoint(highestPointIndex);
                    }

                    var head = new Head(highestPointIndex);
                    var success = head.AddHeadPixels(headPointIndexes);
                    if (success == 1)
                    {
                        validatedCandidateHeads.Add(head);
                    }
                    if (Diagnostics.ShowValidatedHeadPixels)
                    {
                        foreach (var p in head.HeadPointIndexes)
                        {
                            GraphicsUtils.DrawPoint(p);
                        }
                    }
                }
            }
            foreach (var head in validatedCandidateHeads)
            {
                if (GlobVar.SubtractedFilteredPointCloud[head.HighestPointIndex].Z == GlobVar.MaxDepthMeter || GlobVar.SubtractedFilteredPointCloud[head.HighestPointIndex].X == 0 || float.IsInfinity(GlobVar.SubtractedFilteredPointCloud[head.HighestPointIndex].X))
                {
                    
                }
            }

            GlobVar.ValidatedCandidateHeads = validatedCandidateHeads;

            return validatedCandidateHeads;
        }

        private void AnalyzeFrame(TimeSpan frameRelativeTime, CameraSpacePoint[] filteredFrame)
        {
            stopwatch.Restart();

            var validatedHeads = DetectHeads(filteredFrame);

            GlobVar.AdjacancyList = GeodesicUtils.CreateGeodesicGraph(filteredFrame);
            if (Diagnostics.LogTimeGeodesic)
            {
                stopwatch.Stop();
                Console.WriteLine("CreateGeodesicGraph: {0}", stopwatch.ElapsedMilliseconds);
            }
            stopwatch.Restart();

            if (Diagnostics.ShowGeodesicGraph)
            {
                foreach (var v in GlobVar.AdjacancyList)
                {
                    GlobVar.Canvas[v.Key] = 255;
                }
            }

            if (GlobVar.AdjacancyList.Count == 0 && validatedHeads.Count > 0)
            {
                var v = filteredFrame[validatedHeads[0].CenterPointIndex];
            }           

            Dictionary<Head,int> identifiedHeads = BodyUtils.IdentifyHeads(validatedHeads);

            var bodies = new List<Body>();
            foreach (var identifiedHead in identifiedHeads)
            {
                Stopwatch stopwatchBody = new Stopwatch();
                stopwatchBody.Start();

                Body body;

                if (identifiedHead.Value == -1)
                {
                    GlobVar.MaxBodyCount++;
                    body = new Body(GlobVar.MaxBodyCount, frameRelativeTime);
                    body.AddHead(identifiedHead.Key);
                    bodies.Add(body);

                    GeodesicUtils.AddBodyLandmarks(identifiedHead.Key, body);
                    if (Diagnostics.LogTime)
                    {
                        stopwatch.Stop();
                        Console.WriteLine("ShortestPath: {0}", stopwatch.ElapsedMilliseconds);
                    }
                    stopwatch.Restart();
                }
                else
                {
                    int bodyId = identifiedHead.Value;

                    Head head = identifiedHead.Key;
                    head.AvgCenterPoint = BodyUtils.GetAverageHeadLocationLastFrames(bodyId);

                    if (Diagnostics.ShowHeadAvgCenterPoint)
                    {
                        GraphicsUtils.DrawPoint(head.AvgCenterPoint);
                    }

                    body = new Body(bodyId, frameRelativeTime);
                    body.AddHead(head);
                    bodies.Add(body);

                    GeodesicUtils.AddBodyLandmarks(identifiedHead.Key, body);
                    if (Diagnostics.LogTime)
                    {
                        stopwatch.Stop();
                        Console.WriteLine("ShortestPath: {0}", stopwatch.ElapsedMilliseconds);
                    }
                    stopwatch.Restart();
                }
                diagnostics.Draw(body);

                EnergyUtils.UpdateBodyEnergy(body);
                if (Diagnostics.LogTime)
                {
                    stopwatchBody.Stop();
                    Console.WriteLine("ProcessOneBody: {0}", stopwatchBody.ElapsedMilliseconds);
                }
            }

            BodyUtils.UpdateBodyHistory(bodies);
            diagnostics.AddNewBodyFrameDiagnostics(bodies, frameRelativeTime);

            heatMap.AddFrame(GlobVar.HeatCanvas);
            energyUtils.UpdateGlobalEnergy(bodies, frameRelativeTime);

            if (Diagnostics.PrintFPS)
            {
                var currentTime = frameRelativeTime;
                Console.WriteLine("FPS: {0}", GlobUtils.CalculateFPS(currentTime, lastTime));
                lastTime = currentTime;
            }
            
        }

        private void PostProcessing()
        {
            // Calculate tracking success of session
            diagnostics.CalculateTrackingSuccess();

            stopwatch.Restart();
            var statWindow = new StatisticsWindow();
            statWindow.Draw(heatMap.GetMap(), energyUtils.GetEnergyHistory(), diagnostics.Timestamps, diagnostics.BodyTrackedHistory);
            stopwatch.Stop();
            Console.WriteLine("Statwindow: {0}",stopwatch.ElapsedMilliseconds);

        }
    }
}
