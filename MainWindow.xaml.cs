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

        private WriteableBitmap intensityBitmap = new WriteableBitmap(GlobVar.scaledFrameWidth, GlobVar.scaledFrameHeight, 96.0, 96.0, PixelFormats.Gray8, null);
        private byte[] intensityMap = new byte[GlobVar.scaledFrameLength];
        private CameraSpacePoint[] pointCloud = new CameraSpacePoint[GlobVar.scaledFrameLength];

        private CoordinateMapper coordinateMapper = null;

        private CameraSpacePoint[] cameraSpacePoints = null;


        /// <summary>
        /// Map depth range to byte range
        /// </summary>
        private const int MapDepthToByte = 8000 / 256;
        
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
        /// Bitmap to display
        /// </summary>
        private WriteableBitmap depthBitmap = null;

        /// <summary>
        /// Intermediate storage for frame data converted to color
        /// </summary>
        private byte[] depthPixels = null;


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

            // create the bitmap to display
            this.depthBitmap = new WriteableBitmap(this.depthFrameDescription.Width, this.depthFrameDescription.Height, 96.0, 96.0, PixelFormats.Gray8, null);

            // allocate space to put the pixels being received and converted
            this.depthPixels = new byte[this.depthFrameDescription.Width * this.depthFrameDescription.Height * this.depthBitmap.Format.BitsPerPixel / 8];

            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            this.cameraSpacePoints = new CameraSpacePoint[depthFrameDescription.Width * depthFrameDescription.Height];

            // open the sensor
            this.kinectSensor.Open();

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();

        }

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.intensityBitmap;
            }
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

                    stopwatch.Start();


                    GlobVar.scaledCameraSpacePoints = KinectUtils.ScaleFrame(this.cameraSpacePoints);
                    this.pointCloud = KinectUtils.CreatePointCloud(GlobVar.scaledCameraSpacePoints);

                    stopwatch.Stop();
                    Console.WriteLine("pointCloud: {0}", stopwatch.ElapsedMilliseconds);
                    stopwatch.Start();

                    var filteredPointCloud = ImageUtils.MedianFilter3X3(this.pointCloud);
                    
                    stopwatch.Stop();
                    Console.WriteLine("filter: {0}", stopwatch.ElapsedMilliseconds);
                    stopwatch.Start();



                    filteredPointCloud.CopyTo(GlobVar.InvertedCloud, 0);

                    KinectUtils.InvertDistanceMeasures(GlobVar.InvertedCloud);


                    KinectUtils.DepthMapFromCameraPoints(GlobVar.depthMap, GlobVar.InvertedCloud);


                    Point[] candidates = HaarDetector.CreateRegions(GlobVar.depthMap);

                    GlobVar.AdjacancyList = GeodesicUtils.CreateGeodesicGraph();

                    stopwatch.Stop();
                    Console.WriteLine("Haar: {0}", stopwatch.ElapsedMilliseconds);

                    intensityMap = KinectUtils.CalculateIntensityFromCameraSpacePoints(filteredPointCloud);

                    stopwatch.Start();

                    var heads = new List<Head>();

                    for (int i = 0; i < candidates.Length; i++)
                    {
                        var newHead = new Head(200);
                        Point highestPoint = ClassificationUtils.GetHighestPointInSurroundingArea(candidates[i]);

                        ClassificationUtils.ConnectedComponentLabelingIterative(highestPoint, (byte)(200), 150, newHead);
                        BodyUtils.CalculateCenterPointHeadPoints(newHead);

                        heads.Add(newHead);
                        GeodesicUtils.CalculateShortestPaths(newHead);
                    }
                    GlobVar.Heads = heads;
                    

                    stopwatch.Stop();

                    //Graphics.DrawBodies();
                    Console.WriteLine("ConnectComp: {0}", stopwatch.ElapsedMilliseconds);

                    RenderDepthPixels();
                    depthFrameReader.IsPaused = false;
                }
            }
        }

        /// <summary>
        /// Renders color pixels into the writeableBitmap.
        /// </summary>
        private void RenderDepthPixels()
        {
            Graphics.DrawCanvas(intensityMap);
            Graphics.ClearCanvas();

            this.intensityBitmap.WritePixels(
                new Int32Rect(0, 0, this.intensityBitmap.PixelWidth, this.intensityBitmap.PixelHeight),
                this.intensityMap,
                this.intensityBitmap.PixelWidth * intensityBitmap.Format.BitsPerPixel / 8,
                0);

        }
    }
}
