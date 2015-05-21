//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.DepthBasics
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
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        // Optimize by accessing backbuffer of writablebitmap directly


        private const int frameWidth = 512;
        private const int frameHeight = 424;
        private const int frameLength = frameWidth * frameHeight;
        private const int scaledFrameWidth = frameWidth / 2;
        private const int scaledFrameHeight = frameHeight / 2;
        private const int scaledFrameLength = scaledFrameHeight * scaledFrameWidth;

        
        private WriteableBitmap scaledDepthBitmap = new WriteableBitmap(scaledFrameWidth, scaledFrameHeight, 96.0, 96.0, PixelFormats.Gray8, null);
        private CameraSpacePoint[] scaledCameraSpacePoints = new CameraSpacePoint[scaledFrameLength];
        private byte[] scaledDepthPixels = new byte[scaledFrameLength];
        private CameraSpacePoint[] pointCloud = new CameraSpacePoint[scaledFrameLength];

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
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        /// 

        private WriteableBitmap displayBmp = null;

        private VideoStreamSaver writer = null;



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

            writer = new VideoStreamSaver("testSeveral.avi",this.depthFrameDescription.Width, this.depthFrameDescription.Height);
                

            // create the bitmap to display
            this.depthBitmap = new WriteableBitmap(this.depthFrameDescription.Width, this.depthFrameDescription.Height, 96.0, 96.0, PixelFormats.Gray8, null);

            // allocate space to put the pixels being received and converted
            this.depthPixels = new byte[this.depthFrameDescription.Width * this.depthFrameDescription.Height * this.depthBitmap.Format.BitsPerPixel / 8];


            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            this.cameraSpacePoints = new CameraSpacePoint[depthFrameDescription.Width * depthFrameDescription.Height];



            this.displayBmp = new WriteableBitmap(this.depthFrameDescription.Width, this.depthFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgra32, null);

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();


            

        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;
        

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.scaledDepthBitmap;
            }

        }

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            writer.close();
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
                    int a = GlobUtils.getIndex(0,0);
                    int b = GlobUtils.getIndex(10,0);
                    int c = GlobUtils.getIndex(0,10);

                    int area = GlobUtils.calculatePixelAreaFromIndexes(a,b,c);


                    List<IndexRectangle> candidateRects = new List<IndexRectangle>();
                    Stopwatch stopwatch = new Stopwatch();
                    stopwatch.Start();
                    depthFrame.CopyFrameDataToArray(frameData);
                    depthFrameReader.IsPaused = true;
                    this.coordinateMapper.MapDepthFrameToCameraSpace(frameData, this.cameraSpacePoints);
                    stopwatch.Stop();
                    Console.WriteLine("Kinect: {0}", stopwatch.ElapsedMilliseconds);

                    stopwatch.Start();
                    this.scaledCameraSpacePoints = KinectUtils.ScaleFrame(this.cameraSpacePoints);
                    this.pointCloud = KinectUtils.CreatePointCloud(this.scaledCameraSpacePoints);
                    stopwatch.Stop();
                    Console.WriteLine("pointCloud: {0}", stopwatch.ElapsedMilliseconds);
                    stopwatch.Start();
                    this.pointCloud = ImageUtils.MedianFilter3x3(this.pointCloud);
                    stopwatch.Stop();
                    Console.WriteLine("filter: {0}", stopwatch.ElapsedMilliseconds);
                    //ImageUtils.HysteresisThresholding(1, pointCloud);
                    stopwatch.Start();
                    candidateRects = HaarDetector.createRegions(pointCloud);
                    stopwatch.Stop();
                    Console.WriteLine("Haar: {0}", stopwatch.ElapsedMilliseconds);



                    for (int i = 0; i < pointCloud.Length; i++)
                    {
                        this.scaledDepthPixels[i] = KinectUtils.CalculateIntensityFromCameraSpacePoint(pointCloud[i]);
                    }

                    Stopwatch swDraw = new Stopwatch();
                    swDraw.Start();
                    foreach (var rect in candidateRects)
                    {
                        Graphics.DrawRectangle(scaledDepthPixels, rect);
                    }
                    swDraw.Stop();
                    Console.WriteLine("Draw: {0}", swDraw.ElapsedMilliseconds);

                    //ThreePointRectangle rect = new ThreePointRectangle(new Point(5,5),new Point(250,5),new Point(5,205));

                    //Graphics.DrawRectangle(scaledDepthPixels, rect);
                    

                    this.RenderDepthPixels(candidateRects);
                    depthFrameReader.IsPaused = false;
                }
            }
        }

        /// <summary>
        /// Renders color pixels into the writeableBitmap.
        /// </summary>
        private void RenderDepthPixels(List<IndexRectangle> candidateRects)
        {
            Stopwatch swRender = new Stopwatch();
            swRender.Start();
            this.scaledDepthBitmap.WritePixels(
                new Int32Rect(0, 0, this.scaledDepthBitmap.PixelWidth, this.scaledDepthBitmap.PixelHeight),
                this.scaledDepthPixels,
                this.scaledDepthBitmap.PixelWidth * scaledDepthBitmap.Format.BitsPerPixel / 8,
                0);




            swRender.Stop();
            Console.WriteLine("Render: {0}", swRender.ElapsedMilliseconds);


            //this.depthBitmap.WritePixels(
            //    new Int32Rect(0, 0, this.depthBitmap.PixelWidth, this.depthBitmap.PixelHeight),
            //    this.depthPixels,
            //    this.depthBitmap.PixelWidth*depthBitmap.Format.BitsPerPixel/8,
            //    0);
            
            /*
            WriteableBitmap processedBitmap = new WriteableBitmap(detector.ProcessImage(this.depthBitmap,writer));
            byte[] processedPixels = new byte[this.depthFrameDescription.Width * this.depthFrameDescription.Height * processedBitmap.Format.BitsPerPixel / 8];
            
            processedBitmap.CopyPixels(processedPixels, processedBitmap.PixelWidth*processedBitmap.Format.BitsPerPixel/8, 0);

            this.displayBmp.WritePixels(
                new Int32Rect(0, 0, processedBitmap.PixelWidth, processedBitmap.PixelHeight),
                processedPixels,
                processedBitmap.PixelWidth*processedBitmap.Format.BitsPerPixel/8,
                0);
             */
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }

    }
}
