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

        public static CameraSpacePoint[] CreateEmptyPointCloud()
        {
            CameraSpacePoint[] filteredPointCloud = new CameraSpacePoint[GlobVar.ScaledFrameLength];

            for (int i = 0; i < filteredPointCloud.Length; i++)
            {
                filteredPointCloud[i].Z = GlobVar.MaxDepthMeter;
            }
            return filteredPointCloud;
        }
    }


}
