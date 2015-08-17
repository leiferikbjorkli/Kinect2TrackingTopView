using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using Microsoft.Kinect;

namespace InteractionDetection
{
    class TemporalMedianImage
    {
        private Dictionary<int, CameraSpacePoint[]> Frames;
        private CameraSpacePoint[] temporalImage;
        private int FrameCounter;
        private int MaxFrames;
        private bool temporalImageSet;

        public TemporalMedianImage(int frameCount)
        {
            Frames = new Dictionary<int, CameraSpacePoint[]>();
            MaxFrames = frameCount;
            FrameCounter = 0;
            temporalImageSet = false;
        }

        public CameraSpacePoint[] TemporalImage
        {
            get
            {
                if (FrameCounter != MaxFrames)
                {
                    return null;
                }
                return temporalImage;
            }
        }

        public bool ImageSet
        {
            get { return temporalImageSet; }
        }

        public void AddFrame(CameraSpacePoint[] newFrame)
        {
            Frames.Add(FrameCounter,newFrame);
            FrameCounter++;

            if (FrameCounter == MaxFrames)
            {
                CalculateTemporalMedianImage();
            }
        }

        public CameraSpacePoint[] Subtract(CameraSpacePoint[] cameraSpacePoints)
        {
            var subtractedCloud = new CameraSpacePoint[cameraSpacePoints.Length];

            var maxDepth = GlobVar.MaxDepthMeter;

            for (int i = 0; i < cameraSpacePoints.Length; i++)
            {
                CameraSpacePoint cameraSpacePoint = cameraSpacePoints[i];

                if (cameraSpacePoints[i].Z > temporalImage[i].Z - Thresholds.TemporalMedianMapPadding)
                {
                    cameraSpacePoint.Z = maxDepth;
                }

                subtractedCloud[i] = cameraSpacePoint;
            }

            return subtractedCloud;
        }

        private void CalculateTemporalMedianImage()
        {
            temporalImage = new CameraSpacePoint[Frames[1].Length];

            float[] temporalZ = new float[MaxFrames];
            
            for (int j = 0; j < Frames[1].Length; j++)
            {
                for (int i = 0; i < Frames.Count; i++)
                {
                    temporalZ[i] = Frames[i][j].Z;
                }
                Array.Sort(temporalZ);

                TemporalImage[j].Z = temporalZ[0];
            }

            temporalImage = TemporalMedianFilter3X3(temporalImage,4);

            temporalImageSet = true;
        }

        public void Draw(MainWindow mainWindow)
        {
            if (FrameCounter != MaxFrames)
            {
                return;
            }

            byte[] intensityMap = KinectUtils.CalculateIntensityFromCameraSpacePoints(temporalImage);
            GraphicsUtils.DrawCanvas(intensityMap);
            GraphicsUtils.ClearCanvas();
            GlobVar.IntensityBitmap.WritePixels(
                new Int32Rect(0, 0, GlobVar.IntensityBitmap.PixelWidth, GlobVar.IntensityBitmap.PixelHeight),
                intensityMap,
                GlobVar.IntensityBitmap.PixelWidth * GlobVar.IntensityBitmap.Format.BitsPerPixel / 8,
                0);
            XAMLCanvas.DrawCanvas(mainWindow);
        }

        public static CameraSpacePoint[] TemporalMedianFilter3X3(CameraSpacePoint[] pointCloud,int elementNr)
        {
            CameraSpacePoint[] filteredPointCloud = ImageUtils.CreateEmptyPointCloud();

            float[] kernel3x3 = new float[9];
            int frameWidth = GlobVar.ScaledFrameWidth;
            int frameHeight = GlobVar.ScaledFrameHeight;

            for (int i = 1; i < frameHeight - 1; i++)
            {
                for (int j = 1; j < frameWidth - 1; j++)
                {
                    int currentIndex = i * frameWidth + j;

                    kernel3x3[0] = pointCloud[currentIndex].Z;
                    kernel3x3[1] = pointCloud[i * frameWidth + j - 1].Z;
                    kernel3x3[2] = pointCloud[i * frameWidth + j + 1].Z;
                    kernel3x3[3] = pointCloud[(i - 1) * frameWidth + j].Z;
                    kernel3x3[4] = pointCloud[(i + 1) * frameWidth + j].Z;
                    kernel3x3[5] = pointCloud[(i - 1) * frameWidth + j - 1].Z;
                    kernel3x3[6] = pointCloud[(i - 1) * frameWidth + j + 1].Z;
                    kernel3x3[7] = pointCloud[(i + 1) * frameWidth + j - 1].Z;
                    kernel3x3[8] = pointCloud[(i + 1) * frameWidth + j + 1].Z;


                    CameraSpacePoint currentPoint = pointCloud[currentIndex];
                    if (ImageUtils.ContainsSufficientValidData(kernel3x3, elementNr))
                    {
                        Array.Sort(kernel3x3);
                        currentPoint.Z = kernel3x3[4];
                    }
                    else
                    {
                        currentPoint.Z = kernel3x3[0];
                    }
                    filteredPointCloud[currentIndex] = currentPoint;

                }
            }

            return filteredPointCloud;
        }

    }
}
