//
// Copyright (c) Leif Erik Bjoerkli, Norwegian University of Science and Technology, 2015.
// Distributed under the MIT License.
// (See accompanying file LICENSE or copy at http://opensource.org/licenses/MIT)
//

namespace Kinect2TrackingTopView
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Windows;
    using Microsoft.Kinect;

    public partial class MainWindow : Window
    {
        private TimeSpan _timestampPreviousFrame;

        /// <summary>
        /// Points in camera space mapped from depthframe
        /// </summary>
        private readonly CameraSpacePoint[] _cameraSpacePoints;
        
        private KinectSensor _kinectSensor;

        /// <summary>
        /// Reader for depth frames
        /// </summary>
        private DepthFrameReader _depthFrameReader;

        /// <summary>
        /// Description of the data contained in the depth frame
        /// </summary>
        private readonly FrameDescription _depthFrameDescription;

        private readonly HeatMap _heatMap;

        private readonly EnergyHistory _energyHistory;

        private readonly TrackingDiagnostics _trackingDiagnostics;

        private readonly Stopwatch _stopwatch;

        private readonly TemporalMedianImage _temporalMedianImage;

        public MainWindow()
        {
            
            _kinectSensor = KinectSensor.GetDefault();

            _depthFrameDescription = _kinectSensor.DepthFrameSource.FrameDescription;

            _depthFrameReader = _kinectSensor.DepthFrameSource.OpenReader();

            _depthFrameReader.FrameArrived += Reader_FrameArrived;

            _cameraSpacePoints = new CameraSpacePoint[_depthFrameDescription.Width * _depthFrameDescription.Height];

            _trackingDiagnostics = new TrackingDiagnostics();

            _heatMap = new HeatMap();

            _energyHistory = new EnergyHistory();

            _temporalMedianImage = new TemporalMedianImage(GlobVar.TemporalFrameCounter);

            _stopwatch = new Stopwatch();

            BodiesHistory.Initialize();

            GlobVar.CoordinateMapper = _kinectSensor.CoordinateMapper;

            GlobVar.TimeStamps = new List<TimeSpan>();

            // initialize the components (controls) of the GUI window
            InitializeComponent();

            _kinectSensor.Open();
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this._depthFrameReader != null)
            {
                this._depthFrameReader.Dispose();
                this._depthFrameReader = null;
            }

            if (this._kinectSensor != null)
            {
                this._kinectSensor.Close();
                this._kinectSensor = null;
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
            var stopwatchTotalTime = new Stopwatch();
            stopwatchTotalTime.Restart();
            
            ushort[] frameData = new ushort[_depthFrameDescription.Width*_depthFrameDescription.Height];

            using (DepthFrame depthFrame = e.FrameReference.AcquireFrame())
            {
                if (depthFrame != null)
                {
                    _depthFrameReader.IsPaused = true;
                    depthFrame.CopyFrameDataToArray(frameData);
                    GlobVar.CoordinateMapper.MapDepthFrameToCameraSpace(frameData, _cameraSpacePoints);

                    var preProcessedFrame = PreProcessFrame(_cameraSpacePoints);

                    //preProcessedFrame = ImageUtils.DepthFilter(preProcessedFrame);

                    if (_temporalMedianImage.ImageSet)
                    {
                        var subtractedFrame = _temporalMedianImage.Subtract(preProcessedFrame);

                        _stopwatch.Restart();

                        var noiseFilteredFrame = ImageUtils.MedianFilter3X3(subtractedFrame, 2);

                        if (Logger.LogTimeMedian)
                        {
                            _stopwatch.Stop();
                            Console.WriteLine("Median: {0}", _stopwatch.ElapsedMilliseconds);
                        }
                        _stopwatch.Restart();

                        GlobVar.SubtractedFilteredPointCloud = noiseFilteredFrame;

                        TrackBodies(depthFrame.RelativeTime, noiseFilteredFrame);

                        GraphicsUtils.RenderDepthPixels(this, noiseFilteredFrame); 
                        //_temporalMedianImage.Draw(this);
                    }
                    _depthFrameReader.IsPaused = false;
                }
            }
            if (Logger.LogTimeTotal)
            {
                stopwatchTotalTime.Stop();
                Console.WriteLine("TotalTime: {0}", stopwatchTotalTime.ElapsedMilliseconds);
            }

        }

        private CameraSpacePoint[] PreProcessFrame(CameraSpacePoint[] cameraSpacePoints)
        {
            _stopwatch.Restart();

            var preProcessedFrame = new CameraSpacePoint[cameraSpacePoints.Length];

            preProcessedFrame = ImageUtils.ScaleFrame(cameraSpacePoints);
            if (Logger.LogTimeScaling)
            {
                _stopwatch.Stop();
                Console.WriteLine("ScaleFrame: {0}", _stopwatch.ElapsedMilliseconds);
            }
            _stopwatch.Restart();

            if (!_temporalMedianImage.ImageSet)
            {
                _temporalMedianImage.AddFrame(preProcessedFrame);
            }

            return preProcessedFrame;
        }

        /// <summary>
        /// Performs body tracking on frame
        /// </summary>
        private void TrackBodies(TimeSpan frameRelativeTime, CameraSpacePoint[] noiseFilteredFrame)
        {

            var validatedHeads = DetectHeads(noiseFilteredFrame);

            _stopwatch.Restart();

            GeodesicUtils.CreateGeodesicGraph(noiseFilteredFrame);

            if (Logger.LogTimeGeodesic)
            {
                _stopwatch.Stop();
                Console.WriteLine("CreateGeodesicGraph: {0}", _stopwatch.ElapsedMilliseconds);
            }
            _stopwatch.Restart(); 

            var bodies = CreateBodies(frameRelativeTime, validatedHeads);
            BodiesHistory.Update(bodies);

            GlobVar.TimeStamps.Add(frameRelativeTime);
            TrackingDiagnostics.AddNewBodyFrame(bodies);
            _heatMap.AddFrame(GlobVar.HeatCanvas);
            EnergyHistory.Update(bodies);

            if (Logger.PrintFps)
            {
                var currentTime = frameRelativeTime;
                Console.WriteLine("FPS: {0}", Logger.CalculateFps(currentTime, _timestampPreviousFrame));
                _timestampPreviousFrame = currentTime;
            }
        }

        private List<Body> CreateBodies(TimeSpan frameRelativeTime, List<Head> validatedHeads)
        {
            _stopwatch.Restart();
            Dictionary<Head, int> identifiedHeadsWithLabels = BodyUtils.IdentifyHeads(validatedHeads);

            var bodies = new List<Body>();
            foreach (var identifiedHead in identifiedHeadsWithLabels)
            {
                Stopwatch stopwatchBody = new Stopwatch();
                stopwatchBody.Start();

                Body body;
                int bodyId = identifiedHead.Value;
                Head head = identifiedHead.Key;

                if (bodyId == -1)
                {
                    GlobVar.MaxBodyCount++;
                    int newBodyId = GlobVar.MaxBodyCount;
                    body = new Body(newBodyId, frameRelativeTime);
                    body.AddHead(head);
                    bodies.Add(body);

                    BodyUtils.AddBodyRegions(GeodesicUtils.GeodesicGraph, head, body);
                }
                else
                {
                    // Attribute head with average centerpoint in from the last
                    // frames.
                    head.AvgCenterPoint = BodyUtils.GetAverageHeadLocationLastFrames(bodyId);

                    if (Logger.ShowHeadAvgCenterPoint)
                    {
                        GraphicsUtils.DrawPoint(head.AvgCenterPoint);
                    }

                    body = new Body(bodyId, frameRelativeTime);
                    body.AddHead(head);
                    bodies.Add(body);

                    BodyUtils.AddBodyRegions(GeodesicUtils.GeodesicGraph, head, body);
                    
                }
                
                
                Logger.Draw(body);

                EnergyUtils.CalculateBodyMechanicalEnergy(body);
            }

            if (Logger.LogTimeCreateBodies)
            {
                _stopwatch.Stop();
                Console.WriteLine("CreateBodies: {0}", _stopwatch.ElapsedMilliseconds);
            }
            
            return bodies;
        }

        private List<Head> DetectHeads(CameraSpacePoint[] noiseFilteredFrame)
        {
            _stopwatch.Restart();

            var haarDetector = new HaarDetector(noiseFilteredFrame);
            List<int> headCandidatesHaar = haarDetector.GetHeadCandidates();
            if (Logger.LogTimeHaar)
            {
                _stopwatch.Stop();
                Console.WriteLine("Haar: {0}", _stopwatch.ElapsedMilliseconds);
            }
            _stopwatch.Restart();

            var candidatesHighestConnectingPoints = new List<int>();

            for (int i = 0; i < headCandidatesHaar.Count; i++)
            {
                if (Logger.ShowCandidateHeadpixel)
                {
                    GraphicsUtils.DrawPoint(headCandidatesHaar[i]);
                }

                int headCandidateIndex = headCandidatesHaar[i];

                if (headCandidateIndex != -1)
                {
                    int highestPointIndex = ClassificationUtils.GetHighestConnectingPoint(headCandidateIndex, Thresholds.ClassificationHighestPointSearchDepth);

                    candidatesHighestConnectingPoints.Add(highestPointIndex);

                    if (Logger.ShowTopHeadpixel)
                    {
                        GraphicsUtils.DrawPoint(highestPointIndex);
                    }
                }
            }

            var groupedHighestIndexes = ClassificationUtils.GroupCandidatesHighestPoints(candidatesHighestConnectingPoints, Thresholds.ClassificationHighestPointGroupingDistance);

            List<Head> validatedCandidateHeads = ValidateCandidateHeads(groupedHighestIndexes);

            GlobVar.ValidatedCandidateHeads = validatedCandidateHeads;
            if (Logger.LogTimeClassificationValidation)
            {
                _stopwatch.Stop();
                Console.WriteLine("ClassificationValidation: {0}", _stopwatch.ElapsedMilliseconds);
            }
            
            return validatedCandidateHeads;
        }

        private static List<Head> ValidateCandidateHeads(IReadOnlyList<int> groupedHighestIndexes)
        {
            List<Head> validatedCandidateHeads = new List<Head>();

            foreach (var highestPointIndex in groupedHighestIndexes)
            {
                if (Logger.ShowTopHeadPixelAfterGrouping)
                {
                    GraphicsUtils.DrawPoint(highestPointIndex);
                }

                var headPointIndexes = ClassificationUtils.ConnectedComponentLabeling(highestPointIndex, Thresholds.ClassificationLabelingMaxPoints);

                if (Logger.ShowHeadpixelsBeforeValidation)
                {
                    foreach (var index in headPointIndexes)
                    {
                        GraphicsUtils.DrawPoint(index);
                    }
                }

                if (Validators.EvaluateSizeOfHeadRegion(headPointIndexes))
                {
                    if (Logger.ShowValidatedTopHeadpixel)
                    {
                        GraphicsUtils.DrawPoint(highestPointIndex);
                    }

                    var head = new Head(highestPointIndex);
                    var success = head.AddHeadPixels(headPointIndexes);
                    if (success != -1)
                    {
                        validatedCandidateHeads.Add(head);
                    }
                    if (Logger.ShowValidatedHeadPixels)
                    {
                        foreach (var p in head.HeadPointIndexes)
                        {
                            GraphicsUtils.DrawPoint(p);
                        }
                    }
                }
            }
            return validatedCandidateHeads;
        }

        private void PostProcessing()
        {
            if (Logger.PrintTotalFps)
            {
                Console.WriteLine("TotalFPS: {0}", Logger.CalculateAverageFps(GlobVar.TimeStamps));
            }

            TrackingDiagnostics.CalculateTrackingSuccess();

            var statWindow = new StatisticsWindow();
            statWindow.Draw(_heatMap.Get(), _energyHistory.Get(), GlobVar.TimeStamps);
        }
    }
}
