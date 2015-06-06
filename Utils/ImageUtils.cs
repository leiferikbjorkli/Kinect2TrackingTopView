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
        // Optimization: Dont pass large variable to methods, allocate and reclaim new objects quickly, lists 7 times slower than arrays.
        // Measure 
        // Possible to spawn own background thread 
        // Label components recursively

        //public static int[] GetKernelIndexList(int currentNode) {
        //    int[] neighbourIndexes = new int[9];

        //    Point p = GlobUtils.GetPoint(currentNode);
        //    int x = p.x;
        //    int y = p.y;

        //    neighbourIndexes[0] = GlobUtils.GetIndex(x - 1, y - 1);
        //    neighbourIndexes[1] = GlobUtils.GetIndex(x, y - 1);
        //    neighbourIndexes[2] = GlobUtils.GetIndex(x + 1, y - 1);
        //    neighbourIndexes[3] = GlobUtils.GetIndex(x - 1, y + 1);
        //    neighbourIndexes[4] = GlobUtils.GetIndex(x, y + 1);
        //    neighbourIndexes[5] = GlobUtils.GetIndex(x + 1, y + 1);
        //    neighbourIndexes[6] = GlobUtils.GetIndex(x - 1, y);
        //    neighbourIndexes[7] = GlobUtils.GetIndex(x + 1, y);
        //    neighbourIndexes[8] = GlobUtils.GetIndex(x, y);

        //    return neighbourIndexes;
        //}

        //public static CameraSpacePoint[] MedianFilter3X3Simple(CameraSpacePoint[] pointCloud)
        //{
        //    float[] kernel3x3 = new float[9];
        //    int frameWidth = GlobVar.scaledFrameWidth;
        //    int frameHeight = GlobVar.scaledFrameHeight;
        //    CameraSpacePoint[] filteredPointCloud = new CameraSpacePoint[GlobVar.scaledFrameLength];

        //    for (int i = 0; i < pointCloud.Length; i++)
        //    {
        //        kernel3x3 = GetKernelIndexList(i);
        //    }
            
        //    for (int i = 1; i < frameHeight - 1; i++)
        //    {
        //        for (int j = 1; j < frameWidth - 1; j++)
        //        {
        //            kernel3x3[0] = pointCloud[i * frameWidth + j].Z;
        //            kernel3x3[1] = pointCloud[i * frameWidth + j - 1].Z;
        //            kernel3x3[2] = pointCloud[(i * frameWidth + j) + 1].Z;
        //            kernel3x3[3] = pointCloud[(i - 1) * frameWidth + j].Z;
        //            kernel3x3[4] = pointCloud[(i + 1) * frameWidth + j].Z;
        //            kernel3x3[5] = pointCloud[(i - 1) * frameWidth + j - 1].Z;
        //            kernel3x3[6] = pointCloud[(i - 1) * frameWidth + j + 1].Z;
        //            kernel3x3[7] = pointCloud[(i + 1) * frameWidth + j - 1].Z;
        //            kernel3x3[8] = pointCloud[(i + 1) * frameWidth + j + 1].Z;

        //            Array.Sort(kernel3x3);
        //            filteredPointCloud[i * frameWidth + j].Z = kernel3x3[4];
        //            filteredPointCloud[i * frameWidth + j].X = pointCloud[i * frameWidth + j].X;
        //            filteredPointCloud[i * frameWidth + j].Y = pointCloud[i * frameWidth + j].Y;
        //        }
        //    }
        //    return filteredPointCloud;
        //}

        public static CameraSpacePoint[] MedianFilter3X3PointCloud(CameraSpacePoint[] pointCloud)
        {
            float[] kernel3x3 = new float[9];
            int frameWidth = GlobVar.ScaledFrameWidth;
            int frameHeight = GlobVar.ScaledFrameHeight;
            CameraSpacePoint[] filteredPointCloud = new CameraSpacePoint[pointCloud.Length];

            for (int i = 0; i < filteredPointCloud.Length; i++)
            {
                filteredPointCloud[i].Z = GlobVar.MaxDepthMeter;
            }

            for (int i = 1; i < frameHeight - 1; i++)
            {
                for (int j = 1; j < frameWidth - 1; j++)
                {
                    int currentIndex = i*frameWidth + j;
                    CameraSpacePoint currentPoint = pointCloud[currentIndex];

                    kernel3x3[0] = pointCloud[currentIndex].Z;
                    kernel3x3[1] = pointCloud[i * frameWidth + j - 1].Z;
                    kernel3x3[2] = pointCloud[(i * frameWidth + j) + 1].Z;
                    kernel3x3[3] = pointCloud[(i - 1) * frameWidth + j].Z;
                    kernel3x3[4] = pointCloud[(i + 1) * frameWidth + j].Z;
                    kernel3x3[5] = pointCloud[(i - 1) * frameWidth + j - 1].Z;
                    kernel3x3[6] = pointCloud[(i - 1) * frameWidth + j + 1].Z;
                    kernel3x3[7] = pointCloud[(i + 1) * frameWidth + j - 1].Z;
                    kernel3x3[8] = pointCloud[(i + 1) * frameWidth + j + 1].Z;

                    Array.Sort(kernel3x3);
                    filteredPointCloud[currentIndex].Z = kernel3x3[4];
                    filteredPointCloud[currentIndex].X = currentPoint.X;
                    filteredPointCloud[currentIndex].Y = currentPoint.Y;
                }
            }

            return filteredPointCloud;
        }

        public static float[] MedianFilter3X3(CameraSpacePoint[] pointCloud) 
        {
            float[] kernel3x3 = new float[9];
            int frameWidth = GlobVar.ScaledFrameWidth;
            int frameHeight = GlobVar.ScaledFrameHeight;
            float[] filteredPointCloud = new float[pointCloud.Length];

            for (int i = 0; i < filteredPointCloud.Length; i++)
            {
                filteredPointCloud[i] = GlobVar.MaxDepthMeter;
            }

            for (int i = 1; i < frameHeight-1; i++)
            {
                for (int j = 1; j < frameWidth-1; j++)
                {
                    kernel3x3[0] = pointCloud[i * frameWidth + j].Z;
                    kernel3x3[1] = pointCloud[i * frameWidth + j - 1].Z;
                    kernel3x3[2] = pointCloud[(i * frameWidth + j) + 1].Z;
                    kernel3x3[3] = pointCloud[(i - 1) * frameWidth + j].Z;
                    kernel3x3[4] = pointCloud[(i + 1) * frameWidth + j].Z;
                    kernel3x3[5] = pointCloud[(i - 1) * frameWidth + j - 1].Z;
                    kernel3x3[6] = pointCloud[(i - 1) * frameWidth + j + 1].Z;
                    kernel3x3[7] = pointCloud[(i + 1) * frameWidth + j - 1].Z;
                    kernel3x3[8] = pointCloud[(i + 1) * frameWidth + j + 1].Z;

                    Array.Sort(kernel3x3);
                    filteredPointCloud[i * frameWidth + j] = kernel3x3[4];
                }
            }

            return filteredPointCloud;
        }

        public static CameraSpacePoint[] MedianFilter3X3UpperPixel(CameraSpacePoint[] pointCloud)
        {
            float[] kernel3x3 = new float[9];
            int frameWidth = GlobVar.ScaledFrameWidth;
            int frameHeight = GlobVar.ScaledFrameHeight;
            CameraSpacePoint[] filteredPointCloud = new CameraSpacePoint[frameHeight * frameWidth];
            for (int i = 1; i < frameHeight - 1; i++)
            {
                for (int j = 1; j < frameWidth - 1; j++)
                {
                    kernel3x3[0] = pointCloud[i * frameWidth + j].Z;
                    kernel3x3[1] = pointCloud[i * frameWidth + j - 1].Z;
                    kernel3x3[2] = pointCloud[(i * frameWidth + j) + 1].Z;
                    kernel3x3[3] = pointCloud[(i - 1) * frameWidth + j].Z;
                    kernel3x3[4] = pointCloud[(i + 1) * frameWidth + j].Z;
                    kernel3x3[5] = pointCloud[(i - 1) * frameWidth + j - 1].Z;
                    kernel3x3[6] = pointCloud[(i - 1) * frameWidth + j + 1].Z;
                    kernel3x3[7] = pointCloud[(i + 1) * frameWidth + j - 1].Z;
                    kernel3x3[8] = pointCloud[(i + 1) * frameWidth + j + 1].Z;

                    Array.Sort(kernel3x3);
                    filteredPointCloud[i * frameWidth + j].Z = kernel3x3[7];
                    filteredPointCloud[i * frameWidth + j].X = pointCloud[i * frameWidth + j].X;
                    filteredPointCloud[i * frameWidth + j].Y = pointCloud[i * frameWidth + j].Y;
                }
            }
            return filteredPointCloud;
        }

        public static CameraSpacePoint[] MedianFilter5X5alt(CameraSpacePoint[] pointCloud)
        {
            float[] kernel5x5 = new float[25];
            int frameWidth = GlobVar.ScaledFrameWidth;
            int frameHeight = GlobVar.ScaledFrameHeight;
            CameraSpacePoint[] filteredPointCloud = new CameraSpacePoint[frameHeight * frameWidth];
            for (int i = 2; i < frameHeight - 2; i++)
            {
                for (int j = 2; j < frameWidth - 2; j++)
                {
                    int index = 0;
                    for (int k = -2; k < 3; k++)
                    {
                        for (int l = -2; l < 3; l++)
                        {
                            kernel5x5[index] = pointCloud[(i+l) * frameWidth + (j+k)].Z;
                            index++;
                        }
                    }

                    Array.Sort(kernel5x5);
                    filteredPointCloud[i * frameWidth + j].Z = kernel5x5[12];
                }
            }
            return filteredPointCloud;
        }

        public static CameraSpacePoint[] MedianFilter5X5(CameraSpacePoint[] pointCloud)
        {
            float[] kernel5x5 = new float[25];
            int frameWidth = GlobVar.ScaledFrameWidth;
            int frameHeight = GlobVar.ScaledFrameHeight;
            CameraSpacePoint[] filteredPointCloud = new CameraSpacePoint[frameHeight * frameWidth];
            for (int i = 2; i < frameHeight - 2; i++)
            {
                for (int j = 2; j < frameWidth - 2; j++)
                {
                    kernel5x5[0] = pointCloud[i * frameWidth + j].Z;
                    kernel5x5[1] = pointCloud[i * frameWidth + j - 1].Z;
                    kernel5x5[2] = pointCloud[(i * frameWidth + j) + 1].Z;
                    kernel5x5[3] = pointCloud[(i - 1) * frameWidth + j].Z;
                    kernel5x5[4] = pointCloud[(i + 1) * frameWidth + j].Z;
                    kernel5x5[5] = pointCloud[(i - 1) * frameWidth + j - 1].Z;
                    kernel5x5[6] = pointCloud[(i - 1) * frameWidth + j + 1].Z;
                    kernel5x5[7] = pointCloud[(i + 1) * frameWidth + j - 1].Z;
                    kernel5x5[8] = pointCloud[(i + 1) * frameWidth + j + 1].Z;

                    kernel5x5[9] = pointCloud[(i - 2) * frameWidth + j - 2].Z;
                    kernel5x5[10] = pointCloud[(i - 2) * frameWidth + j - 1].Z;
                    kernel5x5[11] = pointCloud[(i - 2) * frameWidth + j].Z;
                    kernel5x5[12] = pointCloud[(i - 2) * frameWidth + j + 1].Z;
                    kernel5x5[13] = pointCloud[(i - 2) * frameWidth + j + 2].Z;

                    kernel5x5[14] = pointCloud[(i + 2) * frameWidth + j - 2].Z;
                    kernel5x5[15] = pointCloud[(i + 2) * frameWidth + j - 1].Z;
                    kernel5x5[16] = pointCloud[(i + 2) * frameWidth + j].Z;
                    kernel5x5[17] = pointCloud[(i + 2) * frameWidth + j + 1].Z;
                    kernel5x5[18] = pointCloud[(i + 2) * frameWidth + j + 2].Z;

                    kernel5x5[19] = pointCloud[(i - 1) * frameWidth + j - 2].Z;
                    kernel5x5[20] = pointCloud[(i - 1) * frameWidth + j + 2].Z;

                    kernel5x5[21] = pointCloud[i * frameWidth + j - 2].Z;
                    kernel5x5[22] = pointCloud[i * frameWidth + j + 2].Z;

                    kernel5x5[23] = pointCloud[(i + 1) * frameWidth + j - 2].Z;
                    kernel5x5[24] = pointCloud[(i + 1) * frameWidth + j + 2].Z;

                    Array.Sort(kernel5x5);
                    filteredPointCloud[i * frameWidth + j].Z = kernel5x5[12];
                }
            }
            return filteredPointCloud;
        }
    }


}
