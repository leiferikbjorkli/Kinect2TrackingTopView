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


        private static int CompareCameraSpacePoints(CameraSpacePoint p1, CameraSpacePoint p2)
        {
            return p1.Z.CompareTo(p2.Z);
        
        }


        public static CameraSpacePoint[] MedianFilter3X3(CameraSpacePoint[] pointCloud) 
        {
            float[] kernel3x3 = new float[9];
            int frameWidth = GlobVar.scaledFrameWidth;
            int frameHeight = GlobVar.scaledFrameHeight;
            CameraSpacePoint[] filteredPointCloud = new CameraSpacePoint[frameHeight*frameWidth];
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
                    filteredPointCloud[i * frameWidth + j].Z = kernel3x3[4];
                    filteredPointCloud[i*frameWidth + j].X = pointCloud[i*frameWidth + j].X;
                    filteredPointCloud[i*frameWidth + j].Y = pointCloud[i*frameWidth + j].Y;
                }
            }
            return filteredPointCloud;
        }

        public static CameraSpacePoint[] MedianFilter3X3UpperPixel(CameraSpacePoint[] pointCloud)
        {
            float[] kernel3x3 = new float[9];
            int frameWidth = GlobVar.scaledFrameWidth;
            int frameHeight = GlobVar.scaledFrameHeight;
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
            int frameWidth = GlobVar.scaledFrameWidth;
            int frameHeight = GlobVar.scaledFrameHeight;
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
            int frameWidth = GlobVar.scaledFrameWidth;
            int frameHeight = GlobVar.scaledFrameHeight;
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

        private static float[] CreateDepthMapFromPointCloud(CameraSpacePoint[] pointCloud)
        {
            float[] depthMap = new float[pointCloud.Length];
            for (int i = 0; i < pointCloud.Length; i++)
            {
                depthMap[i] = pointCloud[i].Z;
            }
            return depthMap;

        }


    }


}
