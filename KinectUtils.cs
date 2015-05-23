using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows;
using System.ComponentModel;
using System.Runtime.InteropServices;
using System.IO;
using System.Drawing;
using System.Drawing.Imaging;
using Microsoft.Kinect;


namespace InteractionDetection
{
    public static class KinectUtils
    {

        private const int maxDepth = 2300;
        private const int minDepth = 1000;
        private const int frameWidth = 512;
        private const int frameHeight = 424;
        private const int frameLength = frameWidth * frameHeight;
        private const int scaledFrameWidth = frameWidth / 2;
        private const int scaledFrameHeight = frameHeight / 2;
        private const int scaledFrameLength = scaledFrameHeight * scaledFrameWidth;
        private const float horizontalFieldOfView = 70.6f;
        private const float verticalFieldOfView = 60.0f;

        public static void DepthMapFromCameraPoints(float[] depthMap, CameraSpacePoint[] cameraSpacePoint)
        {
            for (int i = 0; i < cameraSpacePoint.Length; i++)
            {
                depthMap[i] = cameraSpacePoint[i].Z;
            }
        }


        public static void InvertDistanceMeasures(CameraSpacePoint[] pointCloud)
        {
            for (int i = 0; i < pointCloud.Length; i++)
            {
                if (pointCloud[i].Z != 0)
                {
                    pointCloud[i].Z = (GlobVar.maxDepth / 1000.00f) - pointCloud[i].Z;
                }
            }
        }

        public static CameraSpacePoint[] ScaleFrame(CameraSpacePoint[] cameraSpacePoints)
        { 
            CameraSpacePoint[] newFrame = new CameraSpacePoint[scaledFrameLength];
            int k = 0;
            for (int i = 0; i < frameHeight-1; i+=2)
            {
                for (int j = 0; j < frameWidth-1; j+=2)
                {
                    float sumX = 0;
                    float sumY = 0;
                    float sumZ = 0;
                    int validPixels = 0;

                    CameraSpacePoint upLeft = cameraSpacePoints[(i * frameWidth) + j];
                    float depthCurrent = upLeft.Z;
                    if (!float.IsInfinity(upLeft.X) && !float.IsInfinity(upLeft.Y))
                    {
                        sumX += upLeft.X;
                        sumY += upLeft.Y;
                        sumZ += depthCurrent;
                        validPixels++;
                    }
                    CameraSpacePoint right = cameraSpacePoints[(i * frameWidth) + j + 1];
                    float depthRight = right.Z;
                    if (!float.IsInfinity(right.X) && !float.IsInfinity(right.Y))
                    {
                        sumX += right.X;
                        sumY += right.Y;
                        sumZ += depthRight;
                        validPixels++;
                    }
                    CameraSpacePoint down = cameraSpacePoints[(i * frameWidth + 1) + j];
                    float depthDown = down.Z;
                    if (!float.IsInfinity(down.X) && !float.IsInfinity(down.Y))
                    {
                        sumX += down.X;
                        sumY += down.Y;
                        sumZ += depthDown;
                        validPixels++;
                    }
                    CameraSpacePoint rightDown = cameraSpacePoints[(i * frameWidth + 1) + j + 1];
                    float depthRightDown = rightDown.Z;
                    if (!float.IsInfinity(rightDown.X) && !float.IsInfinity(rightDown.Y))
                    {
                        sumX += rightDown.X;
                        sumY += rightDown.Y;
                        sumZ += depthRightDown;
                        validPixels++;
                    }
                    if (validPixels == 0) { validPixels = 1; };

                    newFrame[k].X = sumX / validPixels;
                    newFrame[k].Y = sumY / validPixels;
                    newFrame[k].Z = sumZ / validPixels;
                    k++;
                }
            }
            return newFrame;
        }


        public static CameraSpacePoint[] CreatePointCloud(CameraSpacePoint[] cameraSpacePoints){

            // Faster to use constants than reading framedescription
            // Faster to declare variable outside loop and reuse memory location
            // Rarely used parts of method should be put into seperate method
            // Faster to calculate maths in a long line than temp storage

            // Optimize by initializing simply types instead of accessing fields
            CameraSpacePoint[] pointCloud = new CameraSpacePoint[cameraSpacePoints.Length];
            double maxHorizontalWidth = Math.Tan((ToRadians(horizontalFieldOfView) / 2)) * maxDepth;
            double maxVerticalHeight = Math.Tan((ToRadians(verticalFieldOfView) / 2)) * maxDepth;
            int frameWidth = scaledFrameWidth;
            int frameHeight = scaledFrameHeight;

            // Optimize by using for instead of for each

            foreach (var cameraSpacePoint in cameraSpacePoints)
            {
                if (!float.IsInfinity(cameraSpacePoint.X) && !float.IsInfinity(cameraSpacePoint.Y) && (int)(cameraSpacePoint.Z * 1000) > minDepth && (int)(cameraSpacePoint.Z * 1000) < maxDepth)
                {
                    int xPixelCoordinateOfPoint = (int)(((cameraSpacePoint.X * 1000) / maxHorizontalWidth) * (frameWidth / 2) + (frameWidth / 2));
                    int yPixelCoordinateOfPoint = (int)(((cameraSpacePoint.Y * 1000) / maxVerticalHeight) * (frameHeight / 2) + (frameHeight / 2));

                    if (xPixelCoordinateOfPoint >= frameWidth)
                    {
                        xPixelCoordinateOfPoint = frameWidth-1;
                    }
                    if (yPixelCoordinateOfPoint > frameHeight)
                    {
                        yPixelCoordinateOfPoint = frameHeight;
                    }

                    if (xPixelCoordinateOfPoint < 1)
                    {
                        xPixelCoordinateOfPoint = 0;
                    }
                    if (yPixelCoordinateOfPoint < 1)
                    {
                        yPixelCoordinateOfPoint = 1;
                    }


                    int pointIndex = ((frameHeight - yPixelCoordinateOfPoint) * frameWidth) + xPixelCoordinateOfPoint;
                    if (cameraSpacePoint.Z < pointCloud[pointIndex].Z || pointCloud[pointIndex].Z == 0)
                    {
                        pointCloud[pointIndex] = cameraSpacePoint;
                    }
                }
            }
            return pointCloud;
        }


        public static Bitmap WriteableBitmapToBitmap(WriteableBitmap writeBitmap)
        {
            Bitmap bmp;
            using (MemoryStream outStream = new MemoryStream())
            {
                BitmapEncoder encoder = new BmpBitmapEncoder();
                encoder.Frames.Add(BitmapFrame.Create(writeBitmap));
                encoder.Save(outStream);
                bmp = new Bitmap(outStream);
            }
            return bmp;

        }

        public static Bitmap ByteArrayToBitmap(byte[] pixelArray)
        {

            MemoryStream ms = new MemoryStream(pixelArray);
            Image image = Image.FromStream(ms);
            Bitmap bitmap = new Bitmap(image);
            return bitmap;
        }

        public static void RemoveParallax(ref CameraSpacePoint[] cameraSpacePointsNoParallax, CameraSpacePoint[] cameraSpacePoints, FrameDescription depthFrameDescription) {


            double depthToProjectionPlane = (depthFrameDescription.Width/2) / Math.Tan(ToRadians(depthFrameDescription.HorizontalFieldOfView/2));

            foreach (var cameraSpacePoint in cameraSpacePoints)
            {
                if (!float.IsInfinity(cameraSpacePoint.X) || !float.IsInfinity(cameraSpacePoint.Y) || !float.IsInfinity(cameraSpacePoint.Z))
                {
                    double angleCenterlineToPointXZ = Math.Atan(cameraSpacePoint.X / cameraSpacePoint.Z);
                    double angleCenterlineToPointYZ = Math.Atan(cameraSpacePoint.Y / cameraSpacePoint.Z);

                    if (Math.Abs(angleCenterlineToPointXZ) > ToRadians(depthFrameDescription.HorizontalFieldOfView / 2))
                    {
                        angleCenterlineToPointXZ = ToRadians(depthFrameDescription.HorizontalFieldOfView / 2) * Math.Sign(angleCenterlineToPointXZ);
                    }

                    if (Math.Abs(angleCenterlineToPointYZ) > ToRadians(depthFrameDescription.VerticalFieldOfView / 2))
                    {
                        angleCenterlineToPointYZ = ToRadians(depthFrameDescription.VerticalFieldOfView / 2) * Math.Sign(angleCenterlineToPointYZ);
                    }

                    int xPixelCoordinateOfPoint = (int)((Math.Tan(angleCenterlineToPointXZ) * depthToProjectionPlane) + (depthFrameDescription.Width / 2));
                    int yPixelCoordinateOfPoint = (int)((Math.Tan(angleCenterlineToPointYZ) * depthToProjectionPlane) + (depthFrameDescription.Height / 2));

                    cameraSpacePointsNoParallax[depthFrameDescription.Width * yPixelCoordinateOfPoint + xPixelCoordinateOfPoint] = cameraSpacePoint;
                    
                }
            }
        }

        private static double ToRadians(double angle) {
            return (Math.PI / 180) * angle;
        }

        public static byte[] CalculateIntensityFromCameraSpacePoints(CameraSpacePoint[] cameraSpacePoint)
        {

            var intensityMap = new byte[cameraSpacePoint.Length];

            for (int i = 0; i < cameraSpacePoint.Length; i++)
            {
                float depthM = cameraSpacePoint[i].Z;
                if (depthM != 0)
                {
                    int depthMM = (int)(depthM * 1000);
                    int currentMax = depthMM - minDepth;
                    int currentDepthRange = maxDepth - minDepth;

                    if (depthMM < maxDepth && depthMM > minDepth)
                    {
                        intensityMap[i] = (byte)(255 - (255 * currentMax / currentDepthRange));
                    }
                    else
                    {
                        intensityMap[i] = (byte)0;
                    }
                   
                }
                else 
                {
                    intensityMap[i] = (byte)0;
                }
            }
            return intensityMap;
        }

        public static byte CalculateIntensityFromDistance(int depth)
            {
            // This will map a distance value to a 0 - 255 range
            // for the purposes of applying the resulting value
            // to RGB pixels.
            int minDepth = 1500;
            int maxDepth = 2230;
            int intervalHeight = (maxDepth-minDepth);

            int currentDepthRange = maxDepth - minDepth;

            int currentMax = depth - minDepth;
            if (depth < maxDepth && depth > minDepth)
                return (byte)(255 - (255 * currentMax / currentDepthRange));
            else
                return (byte)0;



        }


        /// <summary>
        /// Converts a <see cref="System.Drawing.Bitmap"/> into a WPF <see cref="BitmapSource"/>.
        /// </summary>
        /// <remarks>Uses GDI to do the conversion. Hence the call to the marshalled DeleteObject.
        /// </remarks>
        /// <param name="source">The source bitmap.</param>
        /// <returns>A BitmapSource</returns>
        public static BitmapSource ToBitmapSource(this System.Drawing.Bitmap source)
        {
            BitmapSource bitSrc = null;

            var hBitmap = source.GetHbitmap();

            try
            {
                bitSrc = System.Windows.Interop.Imaging.CreateBitmapSourceFromHBitmap(
                    hBitmap,
                    IntPtr.Zero,
                    Int32Rect.Empty,
                    BitmapSizeOptions.FromEmptyOptions());

               

            }
            catch (Win32Exception)
            {
                bitSrc = null;
            }
            finally
            {
                NativeMethods.DeleteObject(hBitmap);
            }
            return bitSrc;

        }

        

        /// <summary>
        /// FxCop requires all Marshalled functions to be in a class called NativeMethods.
        /// </summary>
        internal static class NativeMethods
        {
            [DllImport("gdi32.dll")]
            [return: MarshalAs(UnmanagedType.Bool)]
            internal static extern bool DeleteObject(IntPtr hObject);
        }




    
    }


}
