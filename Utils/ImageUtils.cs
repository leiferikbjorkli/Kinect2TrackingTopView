//
// Written by Leif Erik Bjoerkli
//

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;
using System.Diagnostics;

namespace InteractionDetection
{
    public static class ImageUtils
    {
        public static CameraSpacePoint[] Dilate(CameraSpacePoint[] pointCloud)
        {
            return null;
        }

        public static CameraSpacePoint[] MedianFilter3X3Test(CameraSpacePoint[] pointCloud, int elementNr)
        {
            CameraSpacePoint[] filteredPointCloud = CreateEmptyPointCloud();

            var xValues = new float[pointCloud.Length];
            var yValues = new float[pointCloud.Length];
            var zValues = new float[pointCloud.Length];

            for (int i = 0; i < pointCloud.Length; i++)
            {
                var cameraSpacePoint = pointCloud[i];
                xValues[i] = cameraSpacePoint.X;
                yValues[i] = cameraSpacePoint.Y;
                zValues[i] = cameraSpacePoint.Z;
            }

            var xKernel = new float[9];
            var yKernel = new float[9];
            var zKernel = new float[9];


            for (int i = 1; i < pointCloud.Length - 1; i++)
            {
                var neighbours = GlobUtils.GetNeighbourIndexListFast(i);

                for (int j = 0; j < neighbours.Length; j++)
                {
                    zKernel[j] = zValues[i];
                }
                if (ContainsSufficientValidData(zKernel,elementNr))
                {
                    Array.Sort(zKernel);
                    filteredPointCloud[i].Z = zKernel[4];

                    for (int j = 0; j < neighbours.Length; j++)
                    {
                        xKernel[j] = xValues[i];
                        yKernel[j] = yValues[i];
                    }

                    Array.Sort(xKernel);
                    Array.Sort(yKernel);
                    filteredPointCloud[i].X = xKernel[4];
                    filteredPointCloud[i].Y = yKernel[4];

                }

            }



            return filteredPointCloud;
        }

        public static CameraSpacePoint[] MedianFilter3X3(CameraSpacePoint[] pointCloud, int elementNr)
        {
            CameraSpacePoint[] filteredPointCloud = CreateEmptyPointCloud();

            float[] kernel3x3 = new float[9];
            int frameWidth = GlobVar.ScaledFrameWidth;
            int frameHeight = GlobVar.ScaledFrameHeight;
            float maxDepth = GlobVar.MaxDepthMeter;

            for (int i = 1; i < frameHeight - 1; i++)
            {
                for (int j = 1; j < frameWidth - 1; j++)
                {
                    int currentIndex = i*frameWidth + j;

                    kernel3x3[0] = pointCloud[currentIndex].Z;
                    kernel3x3[1] = pointCloud[i*frameWidth + j - 1].Z;
                    kernel3x3[2] = pointCloud[i*frameWidth + j + 1].Z;
                    kernel3x3[3] = pointCloud[(i - 1)*frameWidth + j].Z;
                    kernel3x3[4] = pointCloud[(i + 1)*frameWidth + j].Z;
                    kernel3x3[5] = pointCloud[(i - 1)*frameWidth + j - 1].Z;
                    kernel3x3[6] = pointCloud[(i - 1)*frameWidth + j + 1].Z;
                    kernel3x3[7] = pointCloud[(i + 1)*frameWidth + j - 1].Z;
                    kernel3x3[8] = pointCloud[(i + 1)*frameWidth + j + 1].Z;


                    CameraSpacePoint currentPoint = pointCloud[currentIndex];
                    if (ContainsSufficientValidData(kernel3x3,elementNr))
                    {
                        Array.Sort(kernel3x3);
                        currentPoint.Z = kernel3x3[elementNr];

                        if (currentPoint.Z != maxDepth)
                        {
                            if (float.IsInfinity(currentPoint.X) || float.IsInfinity(currentPoint.Y))
                            {
                                var validXYValues = GetClosestValidXYValues(currentIndex);
                                currentPoint.X = validXYValues[0];
                                currentPoint.Y = validXYValues[1];
                            }
                        }
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

        private static float[] GetClosestValidXYValues(int currentIndex)
        {
            var neighbours = GlobUtils.GetNeighbourIndexListFast(currentIndex);
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

        public static bool ContainsSufficientValidData(float[] kernel,int elementNr)
        {
            float maxDepth = GlobVar.MaxDepthMeter;

            int count = 0;
            for (int i = 0; i < kernel.Length; i++)
            {
                if (kernel[i] < maxDepth )
                {
                    return true;
                }
                
            }
            return false;
        }

        public static CameraSpacePoint[] MedianFilter5X5(CameraSpacePoint[] pointCloud)
        {
            float[] kernel5x5 = new float[25];
            int frameWidth = GlobVar.ScaledFrameWidth;
            int frameHeight = GlobVar.ScaledFrameHeight;
            CameraSpacePoint[] filteredPointCloud = new CameraSpacePoint[frameHeight*frameWidth];
            for (int i = 2; i < frameHeight - 2; i++)
            {
                for (int j = 2; j < frameWidth - 2; j++)
                {
                    kernel5x5[0] = pointCloud[i*frameWidth + j].Z;
                    kernel5x5[1] = pointCloud[i*frameWidth + j - 1].Z;
                    kernel5x5[2] = pointCloud[(i*frameWidth + j) + 1].Z;
                    kernel5x5[3] = pointCloud[(i - 1)*frameWidth + j].Z;
                    kernel5x5[4] = pointCloud[(i + 1)*frameWidth + j].Z;
                    kernel5x5[5] = pointCloud[(i - 1)*frameWidth + j - 1].Z;
                    kernel5x5[6] = pointCloud[(i - 1)*frameWidth + j + 1].Z;
                    kernel5x5[7] = pointCloud[(i + 1)*frameWidth + j - 1].Z;
                    kernel5x5[8] = pointCloud[(i + 1)*frameWidth + j + 1].Z;

                    kernel5x5[9] = pointCloud[(i - 2)*frameWidth + j - 2].Z;
                    kernel5x5[10] = pointCloud[(i - 2)*frameWidth + j - 1].Z;
                    kernel5x5[11] = pointCloud[(i - 2)*frameWidth + j].Z;
                    kernel5x5[12] = pointCloud[(i - 2)*frameWidth + j + 1].Z;
                    kernel5x5[13] = pointCloud[(i - 2)*frameWidth + j + 2].Z;

                    kernel5x5[14] = pointCloud[(i + 2)*frameWidth + j - 2].Z;
                    kernel5x5[15] = pointCloud[(i + 2)*frameWidth + j - 1].Z;
                    kernel5x5[16] = pointCloud[(i + 2)*frameWidth + j].Z;
                    kernel5x5[17] = pointCloud[(i + 2)*frameWidth + j + 1].Z;
                    kernel5x5[18] = pointCloud[(i + 2)*frameWidth + j + 2].Z;

                    kernel5x5[19] = pointCloud[(i - 1)*frameWidth + j - 2].Z;
                    kernel5x5[20] = pointCloud[(i - 1)*frameWidth + j + 2].Z;

                    kernel5x5[21] = pointCloud[i*frameWidth + j - 2].Z;
                    kernel5x5[22] = pointCloud[i*frameWidth + j + 2].Z;

                    kernel5x5[23] = pointCloud[(i + 1)*frameWidth + j - 2].Z;
                    kernel5x5[24] = pointCloud[(i + 1)*frameWidth + j + 2].Z;

                    Array.Sort(kernel5x5);
                    filteredPointCloud[i*frameWidth + j].Z = kernel5x5[12];
                }
            }
            return filteredPointCloud;
        }

        public static CameraSpacePoint[] CreateEmptyPointCloud()
        {
            CameraSpacePoint[] filteredPointCloud = new CameraSpacePoint[GlobVar.ScaledFrameLength];

            float maxDepth = GlobVar.MaxDepthMeter;

            for (int i = 0; i < filteredPointCloud.Length; i++)
            {
                filteredPointCloud[i] = new CameraSpacePoint()
                {
                    X = float.PositiveInfinity,
                    Y = float.PositiveInfinity,
                    Z = maxDepth
                };
            }
            return filteredPointCloud;
        }

    }


}
