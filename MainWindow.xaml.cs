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
        private readonly CameraSpacePoint[] _cameraSpacePoints = null;
        
        private KinectSensor _kinectSensor = null;

        /// <summary>
        /// Reader for depth frames
        /// </summary>
        private DepthFrameReader _depthFrameReader = null;

        /// <summary>
        /// Description of the data contained in the depth frame
        /// </summary>
        private readonly FrameDescription _depthFrameDescription = null;

        private readonly HeatMap _heatMap;

        private readonly EnergyHistory _energyHistory;

        private readonly TrackingDiagnostics _trackingDiagnostics;

        private readonly Stopwatch _stopwatch;

        private readonly TemporalMedianImage _temporalMedianImage;

        public MainWindow()
        {
            
            _kinectSensor = KinectSensor.GetDefault();

            _depthFrameDescription = _kinectSensor.DepthFrameSource.FrameDescription;
            
            GlobVar.CoordinateMapper = _kinectSensor.CoordinateMapper;

            _depthFrameReader = _kinectSensor.DepthFrameSource.OpenReader();

            _depthFrameReader.FrameArrived += Reader_FrameArrived;

            _cameraSpacePoints = new CameraSpacePoint[_depthFrameDescription.Width * _depthFrameDescription.Height];

            _trackingDiagnostics = new TrackingDiagnostics();

            _heatMap = new HeatMap();

            _energyHistory = new EnergyHistory();

            _temporalMedianImage = new TemporalMedianImage(GlobVar.TemporalFrameCounter);

            _stopwatch = new Stopwatch();

            BodiesHistory.Initialize();

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
            ushort[] frameData = new ushort[_depthFrameDescription.Width*_depthFrameDescription.Height];

            using (DepthFrame depthFrame = e.FrameReference.AcquireFrame())
            {
                if (depthFrame != null)
                {
                    _depthFrameReader.IsPaused = true;
                    depthFrame.CopyFrameDataToArray(frameData);
                    GlobVar.CoordinateMapper.MapDepthFrameToCameraSpace(frameData, this._cameraSpacePoints);

                    var preProcessedFrame = PreProcessFrame(_cameraSpacePoints);

                    if (_temporalMedianImage.ImageSet)
                    {
                        var subtractedFrame = _temporalMedianImage.Subtract(preProcessedFrame);

                        _stopwatch.Restart();

                        var noiseFilteredFrame = ImageUtils.MedianFilter3X3(subtractedFrame,2);

                        if (Debugger.LogTimeMedian)
                        {
                            _stopwatch.Stop();
                            Console.WriteLine("Median: {0}", _stopwatch.ElapsedMilliseconds);
                        }
                        _stopwatch.Restart();

                        GlobVar.SubtractedFilteredPointCloud = noiseFilteredFrame;

                        TrackBodies(depthFrame.RelativeTime, noiseFilteredFrame);

                        GlobVar.PreviousFrame = noiseFilteredFrame;
                        GraphicsUtils.RenderDepthPixels(this, noiseFilteredFrame);

                    }
                    _depthFrameReader.IsPaused = false;
                }
           }
        }

        private CameraSpacePoint[] PreProcessFrame(CameraSpacePoint[] cameraSpacePoints)
        {
            _stopwatch.Restart();

            var preProcessedFrame = new CameraSpacePoint[cameraSpacePoints.Length];

            preProcessedFrame = ImageUtils.ScaleFrame(cameraSpacePoints);
            if (Debugger.LogTime)
            {
                _stopwatch.Stop();
                Console.WriteLine("ScaleFrame: {0}", _stopwatch.ElapsedMilliseconds);
            }
            _stopwatch.Restart();

            GlobVar.ScaledCameraSpacePoints = preProcessedFrame;

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
            _stopwatch.Restart();

            var validatedHeads = DetectHeads(noiseFilteredFrame);

            GeodesicUtils.CreateGeodesicGraph(noiseFilteredFrame);

            if (Debugger.LogTimeGeodesic)
            {
                _stopwatch.Stop();
                Console.WriteLine("CreateGeodesicGraph: {0}", _stopwatch.ElapsedMilliseconds);
            }
            _stopwatch.Restart(); 


            var bodies = CreateBodies(frameRelativeTime, validatedHeads);
            BodiesHistory.Update(bodies);

            _trackingDiagnostics.AddNewBodyFrame(bodies, frameRelativeTime);
            _heatMap.AddFrame(GlobVar.HeatCanvas);
            _energyHistory.Update(bodies, frameRelativeTime);

            if (Debugger.PrintFps)
            {
                var currentTime = frameRelativeTime;
                Console.WriteLine("FPS: {0}", Debugger.CalculateFps(currentTime, _timestampPreviousFrame));
                _timestampPreviousFrame = currentTime;
            }
            
        }

        private List<Body> CreateBodies(TimeSpan frameRelativeTime, List<Head> validatedHeads)
        {
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
                    if (Debugger.LogTime)
                    {
                        _stopwatch.Stop();
                        Console.WriteLine("ShortestPath: {0}", _stopwatch.ElapsedMilliseconds);
                    }
                    _stopwatch.Restart();
                }
                else
                {
                    head.AvgCenterPoint = BodyUtils.GetAverageHeadLocationLastFrames(bodyId);

                    if (Debugger.ShowHeadAvgCenterPoint)
                    {
                        GraphicsUtils.DrawPoint(head.AvgCenterPoint);
                    }

                    body = new Body(bodyId, frameRelativeTime);
                    body.AddHead(head);
                    bodies.Add(body);

                    BodyUtils.AddBodyRegions(GeodesicUtils.GeodesicGraph, head, body);
                    if (Debugger.LogTime)
                    {
                        _stopwatch.Stop();
                        Console.WriteLine("ShortestPath: {0}", _stopwatch.ElapsedMilliseconds);
                    }
                    _stopwatch.Restart();
                }
                Debugger.Draw(body);

                EnergyUtils.UpdateBodyEnergy(body);
                if (Debugger.LogTime)
                {
                    stopwatchBody.Stop();
                    Console.WriteLine("ProcessOneBody: {0}", stopwatchBody.ElapsedMilliseconds);
                }
            }
            return bodies;
        }

        private List<Head> DetectHeads(CameraSpacePoint[] noiseFilteredFrame)
        {
            _stopwatch.Restart();

            var haarDetector = new HaarDetector(noiseFilteredFrame);
            List<int> headCandidatesHaar = haarDetector.GetHeadCandidates();
            if (Debugger.LogTimeHaar)
            {
                _stopwatch.Stop();
                Console.WriteLine("Haar: {0}", _stopwatch.ElapsedMilliseconds);
            }
            _stopwatch.Restart();

            var candidatesHighestConnectingPoints = new List<int>();

            for (int i = 0; i < headCandidatesHaar.Count; i++)
            {
                if (Debugger.ShowCandidateHeadpixel)
                {
                    GraphicsUtils.DrawPoint(headCandidatesHaar[i]);
                }

                int headCandidateIndex = headCandidatesHaar[i];

                if (headCandidateIndex != -1)
                {
                    int highestPointIndex = ClassificationUtils.GetHighestConnectingPoint(headCandidateIndex, Thresholds.ClassificationHighestPointSearchDepth);

                    candidatesHighestConnectingPoints.Add(highestPointIndex);

                    if (Debugger.ShowTopHeadpixel)
                    {
                        GraphicsUtils.DrawPoint(highestPointIndex);
                    }
                }
            }

            var groupedHighestIndexes = ClassificationUtils.GroupCandidatesHighestPoints(candidatesHighestConnectingPoints, Thresholds.ClassificationHighestPointGroupingDistance);

            List<Head> validatedCandidateHeads = ValidateCandidateHeadsHighestPoint(groupedHighestIndexes);

            GlobVar.ValidatedCandidateHeads = validatedCandidateHeads;

            return validatedCandidateHeads;
        }

        private List<Head> ValidateCandidateHeadsHighestPoint(IReadOnlyList<int> groupedHighestIndexes)
        {
            List<Head> validatedCandidateHeads = new List<Head>();

            foreach (var highestPointIndex in groupedHighestIndexes)
            {
                if (Debugger.ShowTopHeadPixelAfterGrouping)
                {
                    GraphicsUtils.DrawPoint(highestPointIndex);
                }

                var headPointIndexes = ClassificationUtils.ConnectedComponentLabeling(highestPointIndex, Thresholds.ClassificationLabelingMaxPoints);

                if (Debugger.ShowHeadpixelsBeforeValidation)
                {
                    foreach (var index in headPointIndexes)
                    {
                        GraphicsUtils.DrawPoint(index);
                    }
                }

                if (Validators.EvaluateSizeOfHeadRegion(headPointIndexes))
                {
                    if (Debugger.ShowValidatedTopHeadpixel)
                    {
                        GraphicsUtils.DrawPoint(highestPointIndex);
                    }

                    var head = new Head(highestPointIndex);
                    var success = head.AddHeadPixels(headPointIndexes);
                    if (success != -1)
                    {
                        validatedCandidateHeads.Add(head);
                    }
                    if (Debugger.ShowValidatedHeadPixels)
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
            _trackingDiagnostics.CalculateTrackingSuccess();

            var statWindow = new StatisticsWindow();
            statWindow.Draw(_heatMap.Get(), _energyHistory.Get(), _trackingDiagnostics.Timestamps, _trackingDiagnostics.HeadTrackedHistory);
        }


    }
}
