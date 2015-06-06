//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace InteractionDetection
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;


    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window
    {
        // Optimize by accessing backbuffer of writablebitmap directly

        private CoordinateMapper coordinateMapper = null;

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

            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            this.cameraSpacePoints = new CameraSpacePoint[depthFrameDescription.Width * depthFrameDescription.Height];

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
                    this.coordinateMapper.MapDepthFrameToCameraSpace(frameData, this.cameraSpacePoints);
                    stopwatch.Stop();
                    //Console.WriteLine("Kinect: {0}", stopwatch.ElapsedMilliseconds);
                    stopwatch.Restart();


                    GlobVar.ScaledCameraSpacePoints = KinectUtils.ScaleFrame(this.cameraSpacePoints);

                    GlobVar.PointCloud = KinectUtils.CreatePointCloud(GlobVar.ScaledCameraSpacePoints);
                    stopwatch.Stop();
                    Console.WriteLine("PointCloud: {0}", stopwatch.ElapsedMilliseconds);
                    stopwatch.Restart();

                    GlobVar.MedianFilteredPointCloud = ImageUtils.MedianFilter3X3PointCloud(GlobVar.PointCloud);
                    stopwatch.Stop();
                    Console.WriteLine("MedianFilter: {0}", stopwatch.ElapsedMilliseconds);
                    stopwatch.Restart();

                    GlobVar.AdjacancyList = GeodesicUtils.CreateGeodesicGraph(GlobVar.MedianFilteredPointCloud);
                    stopwatch.Stop();
                    Console.WriteLine("GeodesicGraph: {0}", stopwatch.ElapsedMilliseconds);
                    stopwatch.Restart();

                    //foreach (var v in GlobVar.AdjacancyList)
                    //{
                    //    GlobVar.Canvas[v.Key] = 255;
                    //}

                    Point[] candidates = HaarDetector.CreateRegions(GlobVar.MedianFilteredPointCloud);
                    stopwatch.Stop();
                    Console.WriteLine("Haar: {0}", stopwatch.ElapsedMilliseconds);
                    stopwatch.Restart();


                    var bodies = new List<Body>();
                    for (int i = 0; i < candidates.Length; i++)
                    {

                        int highestPointIndex = ClassificationUtils.GetHighestPointInSurroundingArea(candidates[i]);
                        stopwatch.Stop();
                        Console.WriteLine("HighestPointInArea: {0}", stopwatch.ElapsedMilliseconds);
                        stopwatch.Restart();

                        var newHead = new Head();
                        var success = ClassificationUtils.ConnectedComponentLabelingIterative(highestPointIndex, 150, newHead);
                        stopwatch.Stop();
                        Console.WriteLine("Connected comp: {0}", stopwatch.ElapsedMilliseconds);
                        stopwatch.Restart();
                        if (success == 1)
                        {
                            //foreach (var headPixel in newHead.HeadPixels) {
                            //    GlobVar.Canvas[headPixel] = 180;
                            //}

                            //Graphics.DrawPoint(GlobUtils.GetPoint(newHead.CenterPoint));

                            Body newBody = new Body(newHead);
                            GeodesicUtils.CalculateShortestPaths(newHead, newBody);
                            bodies.Add(newBody);
                            stopwatch.Stop();
                            Console.WriteLine("ShortestPath: {0}", stopwatch.ElapsedMilliseconds);
                        }
                    }
                    GlobVar.Bodies = bodies;

                    Graphics.RenderDepthPixels(this);
                    depthFrameReader.IsPaused = false;
                }
            }
        }

    }
}
