//
// Copyright (c) Leif Erik Bjoerkli, Norwegian University of Science and Technology, 2015.
// Distributed under the MIT License.
// (See accompanying file LICENSE or copy at http://opensource.org/licenses/MIT)
//  

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using Microsoft.Kinect;

namespace Kinect2TrackingTopView
{
    /// <summary>
    /// Class that saves background frames in an initial phase and calculates a temporal median-filtered background image that is subtracted from later frames.
    /// </summary>
    class TemporalMedianImage
    {
        private readonly Dictionary<int, CameraSpacePoint[]> _temporalFrames;
        private CameraSpacePoint[] _temporalImage;
        private int _frameCounter;
        private readonly int _maxTemporalFrames;

        public TemporalMedianImage(int frameCount)
        {
            _temporalFrames = new Dictionary<int, CameraSpacePoint[]>();
            _maxTemporalFrames = frameCount;
            _frameCounter = 0;
            ImageSet = false;
        }

        public bool ImageSet { get; private set; }

        public void AddFrame(CameraSpacePoint[] newFrame)
        {
            _temporalFrames.Add(_frameCounter,newFrame);
            _frameCounter++;

            if (_frameCounter == _maxTemporalFrames)
            {
                CalculateTemporalMedianImage();
            }
        }

        /// <summary>
        /// Subtracts the pixels where the depth value is deeper than the respective depth value in the constructed background frame.
        /// </summary>
        public CameraSpacePoint[] Subtract(CameraSpacePoint[] cameraSpacePoints)
        {
            var subtractedImage = new CameraSpacePoint[cameraSpacePoints.Length];

            var maxDepth = GlobVar.MaxSensingDepth;

            for (int i = 0; i < cameraSpacePoints.Length; i++)
            {
                CameraSpacePoint cameraSpacePoint = cameraSpacePoints[i];

                if (cameraSpacePoints[i].Z > _temporalImage[i].Z - Thresholds.TemporalMedianMapPadding)
                {
                    cameraSpacePoint.Z = maxDepth;
                }

                subtractedImage[i] = cameraSpacePoint;
            }

            return subtractedImage;
        }

        /// <summary>
        /// Sorts the depth values at each pixel from the frames collected in the background collection phase. The shallowest pixel is stored in a new constructed frame for the background. Noise is removed from the constructed background frame with a 3x3 kernel median filter.
        /// </summary>
        private void CalculateTemporalMedianImage()
        {
            _temporalImage = new CameraSpacePoint[_temporalFrames[1].Length];

            float[] temporalZ = new float[_maxTemporalFrames];
            
            for (int j = 0; j < _temporalFrames[1].Length; j++)
            {
                for (int i = 0; i < _temporalFrames.Count; i++)
                {
                    temporalZ[i] = _temporalFrames[i][j].Z;
                }
                Array.Sort(temporalZ);

                // Save shallowest point from the initial phase
                _temporalImage[j].Z = temporalZ[0];
            }

            _temporalImage = TemporalMedianFilter3X3(_temporalImage,4);

            ImageSet = true;
        }
        /// <summary>
        /// Helper function to draw constructed background frame.
        /// </summary>
        public void Draw(MainWindow mainWindow)
        {
            if (_frameCounter != _maxTemporalFrames)
            {
                return;
            }

            byte[] intensityMap = ImageUtils.CalculateIntensityFromCameraSpacePoints(_temporalImage);
            GraphicsUtils.DrawCanvas(intensityMap);
            GraphicsUtils.ClearCanvas();
            GlobVar.IntensityBitmap.WritePixels(
                new Int32Rect(0, 0, GlobVar.IntensityBitmap.PixelWidth, GlobVar.IntensityBitmap.PixelHeight),
                intensityMap,
                GlobVar.IntensityBitmap.PixelWidth * GlobVar.IntensityBitmap.Format.BitsPerPixel / 8,
                0);
            XamlCanvas.Draw(mainWindow);
        }

        /// <summary>
        /// Processes the temporal image with a median filter with a 3x3 kernel to remove noise.
        /// </summary>
        /// /// <remarks> 
        /// Only depth values are stored in the output image.
        /// </remarks>
        private static CameraSpacePoint[] TemporalMedianFilter3X3(CameraSpacePoint[] pointCloud,int elementNr)
        {
            CameraSpacePoint[] filteredPointCloud = ImageUtils.CreateEmptyPointCloud();

            float[] kernel3X3 = new float[9];
            int frameWidth = GlobVar.ScaledFrameWidth;
            int frameHeight = GlobVar.ScaledFrameHeight;

            for (int i = 1; i < frameHeight - 1; i++)
            {
                for (int j = 1; j < frameWidth - 1; j++)
                {
                    int currentIndex = i * frameWidth + j;

                    kernel3X3[0] = pointCloud[currentIndex].Z;
                    kernel3X3[1] = pointCloud[i * frameWidth + j - 1].Z;
                    kernel3X3[2] = pointCloud[i * frameWidth + j + 1].Z;
                    kernel3X3[3] = pointCloud[(i - 1) * frameWidth + j].Z;
                    kernel3X3[4] = pointCloud[(i + 1) * frameWidth + j].Z;
                    kernel3X3[5] = pointCloud[(i - 1) * frameWidth + j - 1].Z;
                    kernel3X3[6] = pointCloud[(i - 1) * frameWidth + j + 1].Z;
                    kernel3X3[7] = pointCloud[(i + 1) * frameWidth + j - 1].Z;
                    kernel3X3[8] = pointCloud[(i + 1) * frameWidth + j + 1].Z;

                    CameraSpacePoint currentPoint = pointCloud[currentIndex];
                    if (ImageUtils.ContainsValidData(kernel3X3))
                    {
                        Array.Sort(kernel3X3);
                        currentPoint.Z = kernel3X3[4];
                    }
                    else
                    {
                        currentPoint.Z = kernel3X3[0];
                    }
                    filteredPointCloud[currentIndex] = currentPoint;
                }
            }
            return filteredPointCloud;
        }
    }
}
