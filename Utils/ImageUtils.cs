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
using Microsoft.Kinect;
using System.Diagnostics;

namespace Kinect2TrackingTopView
{
    public static class ImageUtils
    {
        /// <summary>
        /// Processes the input point cloud with a median filter with a 3x3 kernel to remove noise. 
        /// </summary>
        /// <param name="pointCloud"></param>
        /// <param name="elementNr">Specifies the index of the pixel in the sorted kernel that is chosen.</param>
        /// <remarks>
        /// To avoid unnecessary computational load, the method checks if the kernel contains useful data before sorting.
        /// </remarks>
        public static CameraSpacePoint[] MedianFilter3X3(CameraSpacePoint[] pointCloud, int elementNr)
        {
            CameraSpacePoint[] filteredPointCloud = CreateEmptyPointCloud();

            float[] kernel3X3 = new float[9];
            int frameWidth = GlobVar.ScaledFrameWidth;
            int frameHeight = GlobVar.ScaledFrameHeight;
            float maxDepth = GlobVar.MaxSensingDepth;

            for (int i = 1; i < frameHeight - 1; i++)
            {
                for (int j = 1; j < frameWidth - 1; j++)
                {
                    int currentIndex = i*frameWidth + j;

                    kernel3X3[0] = pointCloud[currentIndex].Z;
                    kernel3X3[1] = pointCloud[i*frameWidth + j - 1].Z;
                    kernel3X3[2] = pointCloud[i*frameWidth + j + 1].Z;
                    kernel3X3[3] = pointCloud[(i - 1)*frameWidth + j].Z;
                    kernel3X3[4] = pointCloud[(i + 1)*frameWidth + j].Z;
                    kernel3X3[5] = pointCloud[(i - 1)*frameWidth + j - 1].Z;
                    kernel3X3[6] = pointCloud[(i - 1)*frameWidth + j + 1].Z;
                    kernel3X3[7] = pointCloud[(i + 1)*frameWidth + j - 1].Z;
                    kernel3X3[8] = pointCloud[(i + 1)*frameWidth + j + 1].Z;

                    CameraSpacePoint currentPoint = pointCloud[currentIndex];
                    if (ContainsValidData(kernel3X3))
                    {
                        Array.Sort(kernel3X3);
                        currentPoint.Z = kernel3X3[elementNr];

                        if (currentPoint.Z != maxDepth)
                        {
                            if (float.IsInfinity(currentPoint.X) || float.IsInfinity(currentPoint.Y))
                            {
                                var validXyValues = GetClosestValidXyValues(currentIndex);
                                currentPoint.X = validXyValues[0];
                                currentPoint.Y = validXyValues[1];
                            }
                        }
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

        /// <summary>
        /// Makes sure the filtered pixel also has valid x- and y-coordinates.
        /// </summary>
        /// <remarks>
        /// Helper function to the 3x3 median filter. 
        /// </remarks>
        private static float[] GetClosestValidXyValues(int currentIndex)
        {
            var neighbours = GlobUtils.GetNeighbour3X3IndexList(currentIndex);
            var validValues = new float[]{float.PositiveInfinity,float.PositiveInfinity};

            for (int i = 0; i < neighbours.Length; i++)
            {
                var point = GlobVar.SubtractedFilteredPointCloud[i];
                var x = point.X;
                var y = point.Y;

                if (!float.IsInfinity(x) && !float.IsInfinity(y))
                {
                    validValues[0] = x;
                    validValues[1] = y;
                }
            }
            return validValues;
        }

        /// <summary>
        /// Checks if the kernel contains valid data.
        /// </summary>
        /// /// <remarks>
        /// Helper function to the 3x3 median filter. 
        /// </remarks>
        public static bool ContainsValidData(float[] kernel)
        {
            float maxDepth = GlobVar.MaxSensingDepth;

            for (int i = 0; i < kernel.Length; i++)
            {
                if (kernel[i] < maxDepth )
                {
                    return true;
                }
            }
            return false;
        }

        public static CameraSpacePoint[] CreateEmptyPointCloud()
        {
            CameraSpacePoint[] emptyPointCloud = new CameraSpacePoint[GlobVar.ScaledFrameLength];

            float maxDepth = GlobVar.MaxSensingDepth;

            for (int i = 0; i < emptyPointCloud.Length; i++)
            {
                emptyPointCloud[i] = new CameraSpacePoint()
                {
                    X = float.PositiveInfinity,
                    Y = float.PositiveInfinity,
                    Z = maxDepth
                };
            }
            return emptyPointCloud;
        }

        /// <summary>
        /// Scales the input frame to a fourth of the size of the input frame.
        /// </summary>
        /// <remarks>
        /// The scaling pixel region is checked for valid pixels and the average of the valid pixels are used in the output frame.
        /// </remarks>
        public static CameraSpacePoint[] ScaleFrame(CameraSpacePoint[] cameraSpacePoints)
        {
            CameraSpacePoint[] newFrame = new CameraSpacePoint[GlobVar.ScaledFrameLength];
            int pixelCounter = 0;

            const int frameHeight = GlobVar.FrameHeight;
            const int frameWidth = GlobVar.FrameWidth;
            const float maxDepth = GlobVar.MaxSensingDepth;

            for (int i = 0; i < frameHeight - 1; i += 2)
            {
                for (int j = 0; j < frameWidth - 1; j += 2)
                {
                    float sumX = 0;
                    float sumY = 0;
                    float sumZ = 0;
                    int validPixels = 0;

                    CameraSpacePoint upLeft = cameraSpacePoints[(i * frameWidth) + j];
                    float depthCurrent = upLeft.Z;
                    if (!float.IsInfinity(depthCurrent))
                    {
                        sumX += upLeft.X;
                        sumY += upLeft.Y;
                        sumZ += depthCurrent;
                        validPixels++;
                    }
                    CameraSpacePoint right = cameraSpacePoints[(i * frameWidth) + j + 1];
                    float depthRight = right.Z;
                    if (!float.IsInfinity(depthRight))
                    {
                        sumX += right.X;
                        sumY += right.Y;
                        sumZ += depthRight;
                        validPixels++;
                    }
                    CameraSpacePoint down = cameraSpacePoints[(i * frameWidth + 1) + j];
                    float depthDown = down.Z;
                    if (!float.IsInfinity(depthDown))
                    {
                        sumX += down.X;
                        sumY += down.Y;
                        sumZ += depthDown;
                        validPixels++;
                    }
                    CameraSpacePoint rightDown = cameraSpacePoints[(i * frameWidth + 1) + j + 1];
                    float depthRightDown = rightDown.Z;
                    if (!float.IsInfinity(depthRightDown))
                    {
                        sumX += rightDown.X;
                        sumY += rightDown.Y;
                        sumZ += depthRightDown;
                        validPixels++;
                    }
                    if (validPixels == 0)
                    {
                        newFrame[pixelCounter] = new CameraSpacePoint()
                        {
                            X = float.PositiveInfinity,
                            Y = float.PositiveInfinity,
                            Z = maxDepth
                        };
                    }
                    else
                    {
                        newFrame[pixelCounter].X = sumX / validPixels;
                        newFrame[pixelCounter].Y = sumY / validPixels;
                        newFrame[pixelCounter].Z = sumZ / validPixels;
                    }
                    pixelCounter++;
                }
            }
            return newFrame;
        }

        /// <summary>
        ///  Normalizes the depths in the input frame into a 0-255 byte gray-value array.
        /// </summary>
        public static byte[] CalculateIntensityFromCameraSpacePoints(CameraSpacePoint[] cameraSpacePoint)
        {
            var intensityMap = new byte[cameraSpacePoint.Length];

            for (int i = 0; i < cameraSpacePoint.Length; i++)
            {
                float depth = cameraSpacePoint[i].Z;
                if (depth != 0)
                {
                    float currentMax = depth - GlobVar.MinSensingDepth;
                    const float currentDepthRange = GlobVar.MaxSensingDepth - GlobVar.MinSensingDepth;

                    if (depth < GlobVar.MaxSensingDepth && depth > GlobVar.MinSensingDepth)
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
    }
}
