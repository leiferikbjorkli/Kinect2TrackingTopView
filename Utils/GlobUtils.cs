//
// Copyright (c) Leif Erik Bjoerkli, Norwegian University of Science and Technology, 2015.
// Distributed under the MIT License.
// (See accompanying file LICENSE or copy at http://opensource.org/licenses/MIT)
//  

using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;

namespace Kinect2TrackingTopView
{
    static class GlobUtils
    {
        public static int GetIndex(int x, int y)
        {
            if (x < 0 || x > 255 || y < 0 || y > 211)
            {
                return -1;
            }

            return y*GlobVar.ScaledFrameWidth + x;
        }

        public static int GetIndex(Point p)
        {
            int x = p.X;
            int y = p.Y;
            if (x < 0 || x > 255 || y < 0 || y > 211)
            {
                return -1;
            }

            return y*GlobVar.ScaledFrameWidth + x;
        }

        public static Point GetPoint(int index)
        {
            if (index < 0 || index > GlobVar.ScaledFrameLength - 1)
            {
                throw new System.ArgumentException("Index out of bounds");
            }
            Point p;
            p.X = index%GlobVar.ScaledFrameWidth;
            double yDouble = index/GlobVar.ScaledFrameWidth;
            int y = (int) Math.Floor(yDouble);
            p.Y = y;
            return p;
        }

        public static int CalculateRectangleAreaFromIndexes(int a, int b, int c)
        {
            return (GetPoint(b).X - GetPoint(a).X)*(GetPoint(c).Y - GetPoint(a).Y);
        }

        public static float GetEuclideanDistance(int indexA, int indexB)
        {
            CameraSpacePoint a = GlobVar.SubtractedFilteredPointCloud[indexA];
            CameraSpacePoint b = GlobVar.SubtractedFilteredPointCloud[indexB];

            float distance =
                (float) Math.Sqrt((a.X - b.X)*(a.X - b.X) + (a.Y - b.Y)*(a.Y - b.Y) + (a.Z - b.Z)*(a.Z - b.Z));

            if (float.IsNaN(distance))
            {
                throw new System.ArgumentException("distance NaN");
            }
            return distance;
        }

        public static float GetEuclideanDistance(CameraSpacePoint a, CameraSpacePoint b)
        {
            float distance =
                (float)Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y) + (a.Z - b.Z) * (a.Z - b.Z));

            return float.IsNaN(distance) ? float.MaxValue : distance;
        }

        public static float GetEuclideanDistanceXYPlane(int aIndex, int bIndex)
        {
            var a = GlobVar.SubtractedFilteredPointCloud[aIndex];
            var b = GlobVar.SubtractedFilteredPointCloud[bIndex];

            float distance =
                (float)Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y));

            return float.IsNaN(distance) ? float.MaxValue : distance;
        }

        public static bool BoundaryCheck(int index)
        {
            return index > 0 && index < GlobVar.ScaledFrameLength;
        }

        public static float GetHeightDifference(CameraSpacePoint a, CameraSpacePoint b)
        {
            return Math.Abs(a.Z - b.Z);
        }

        public static int[] GetNeighbour3X3IndexList(int currentNode)
        {
            var neighbourIndexes = new int[8];

            Point p = GetPoint(currentNode);
            int x = p.X;
            int y = p.Y;

            neighbourIndexes[0] = GetIndex(x - 1, y - 1);
            neighbourIndexes[1] = GetIndex(x, y - 1);
            neighbourIndexes[2] = GetIndex(x + 1, y - 1);
            neighbourIndexes[3] = GetIndex(x - 1, y + 1);
            neighbourIndexes[4] = GetIndex(x, y + 1);
            neighbourIndexes[5] = GetIndex(x + 1, y + 1);
            neighbourIndexes[6] = GetIndex(x - 1, y);
            neighbourIndexes[7] = GetIndex(x + 1, y); 
 
            return neighbourIndexes;
        }

        public static int[] GetNeighbour5X5IndexList(int currentNode)
        {
            int[] neighbourIndexes = new int[24];

            var frameWidth = GlobVar.ScaledFrameWidth;

            Point p = GetPoint(currentNode);
            int x = p.X;
            int y = p.Y;

            
            neighbourIndexes[1] = y*frameWidth + x - 1;
            neighbourIndexes[2] = (y*frameWidth + x) + 1;
            neighbourIndexes[3] = (y - 1)*frameWidth + x;
            neighbourIndexes[4] = (y + 1)*frameWidth + x;
            neighbourIndexes[5] = (y - 1)*frameWidth + x - 1;
            neighbourIndexes[6] = (y - 1)*frameWidth + x + 1;
            neighbourIndexes[7] = (y + 1)*frameWidth + x - 1;
            neighbourIndexes[8] = (y + 1)*frameWidth + x + 1;

            neighbourIndexes[9] = (y - 2)*frameWidth + x - 2;
            neighbourIndexes[10] = (y - 2)*frameWidth + x - 1;
            neighbourIndexes[11] = (y - 2)*frameWidth + x;
            neighbourIndexes[12] = (y - 2)*frameWidth + x + 1;
            neighbourIndexes[13] = (y - 2)*frameWidth + x + 2;

            neighbourIndexes[14] = (y + 2)*frameWidth + x - 2;
            neighbourIndexes[15] = (y + 2)*frameWidth + x - 1;
            neighbourIndexes[16] = (y + 2)*frameWidth + x;
            neighbourIndexes[17] = (y + 2)*frameWidth + x + 1;
            neighbourIndexes[18] = (y + 2)*frameWidth + x + 2;

            neighbourIndexes[19] = (y - 1)*frameWidth + x - 2;
            neighbourIndexes[20] = (y - 1)*frameWidth + x + 2;

            neighbourIndexes[21] = y*frameWidth + x - 2;
            neighbourIndexes[22] = y*frameWidth + x + 2;

            neighbourIndexes[23] = (y + 1)*frameWidth + x - 2;
            neighbourIndexes[0] = (y + 1) * frameWidth + x + 2;

            return neighbourIndexes;
        }

        public static double ToRadians(double angle)
        {
            return (Math.PI / 180) * angle;
        }

        public static int GetHighestValidPointIndexInRectangle(int bIndex, int cIndex)
        {
            Point pB = GetPoint(bIndex);
            Point pC = GetPoint(cIndex);
            int rectangleWidth = pB.X - pC.X;

            CameraSpacePoint highestPoint = new CameraSpacePoint()
            {
                X = float.PositiveInfinity,
                Y = float.PositiveInfinity,
                Z = float.PositiveInfinity
            };

            int highestPointIndex = -1;

            for (int i = 0; i < rectangleWidth; i++)
            {
                for (int j = 0; j < rectangleWidth; j++)
                {
                    int currentIndex = GetIndex(pC.X + j, pB.Y + i);
                    if (BoundaryCheck(currentIndex))
                    {
                        CameraSpacePoint currentPoint = GlobVar.SubtractedFilteredPointCloud[currentIndex];

                        if (currentPoint.Z < highestPoint.Z && !float.IsInfinity(currentPoint.X) && !float.IsInfinity(currentPoint.Y))
                        {
                            highestPoint = currentPoint;
                            highestPointIndex = currentIndex;
                        }
                    }
                }
            }
            return highestPointIndex;
        }
    }
}