﻿//
// Written by Leif Erik Bjoerkli
//


using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;

namespace InteractionDetection
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
            int x = p.x;
            int y = p.y;
            if (x < 0 || x > 255 || y < 0 || y > 211)
            {
                throw new System.ArgumentException("Coordinates are incorrect");
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
            p.x = index%GlobVar.ScaledFrameWidth;
            double yDouble = index/GlobVar.ScaledFrameWidth;
            int y = (int) Math.Floor(yDouble);
            p.y = y;
            return p;
        }

        public static int CalculatePixelAreaFromIndexes(int a, int b, int c)
        {
            return (GetPoint(b).x - GetPoint(a).x)*(GetPoint(c).y - GetPoint(a).y);
        }

        public static float GetEuclideanDistance(int indexA, int indexB)
        {
            CameraSpacePoint a = GlobVar.PointCloud[indexA];
            CameraSpacePoint b = GlobVar.PointCloud[indexB];

            float distance =
                (float) Math.Sqrt((a.X - b.X)*(a.X - b.X) + (a.Y - b.Y)*(a.Y - b.Y) + (a.Z - b.Z)*(a.Z - b.Z));

            if (float.IsNaN(distance))
            {
                throw new System.ArgumentException("distance NaN");
            }
            return distance;
        }

        public static float GetEuclideanDistance(Point a, Point b)
        {
            CameraSpacePoint aCameraSpace = GlobVar.PointCloud[GetIndex(a)];
            CameraSpacePoint bCameraSpace = GlobVar.PointCloud[GetIndex(b)];

            return (float)
                Math.Sqrt((aCameraSpace.X - bCameraSpace.X)*(aCameraSpace.X - bCameraSpace.X) +
                          (aCameraSpace.Y - bCameraSpace.Y)*(aCameraSpace.Y - bCameraSpace.Y) +
                          (aCameraSpace.Z - bCameraSpace.Z)*(aCameraSpace.Z - bCameraSpace.Z));
        }

        public static float GetEuclideanDistance(CameraSpacePoint a, CameraSpacePoint b)
        {

            float distance =
                (float)Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y) + (a.Z - b.Z) * (a.Z - b.Z));

            if (float.IsNaN(distance))
            {
                return float.MaxValue;
            }
            return distance;
        }

        public static int GetRectangleCenterPointIndex(IndexRectangle rect)
        {
            var a = GetPoint(rect.a);
            var b = GetPoint(rect.b);
            var c = GetPoint(rect.c);

            return GetIndex(a.x + (b.x - a.x)/2, a.y + (c.y - a.y)/2);
        }

        public static Point GetMidPointFromIndexes(int aIndex, int bIndex)
        {
            var a = GetPoint(aIndex);
            var b = GetPoint(bIndex);

            return new Point((a.x + b.x)/2, (a.y + b.y)/2);
        }

        public static float GetDistanceInFrame(Point a, Point b)
        {
            return (float) Math.Sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
        }

        public static float GetDistanceInFrame(int indexA, int indexB)
        {
            Point a = GetPoint(indexA);
            Point b = GetPoint(indexB);
            return (float)Math.Sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
        }

        public static bool BoundaryCheck(Point p)
        {
            int x = p.x;
            int y = p.y;

            return x >= 0 && x <= 255 && y >= 0 && y <= 211;
        }

        public static bool BoundaryCheck(int x, int y)
        {
            return x >= 0 && x <= 255 && y >= 0 && y <= 211;
        }

        public static bool BoundaryCheck(int index)
        {
            return index > 0 && index < GlobVar.ScaledFrameLength;
        }

        public static bool IsNeighbour(int nodeA, int nodeB)
        {
            Point a = GetPoint(nodeA);
            Point b = GetPoint(nodeB);
            int diffX = Math.Abs(b.x - a.x);
            int diffY = Math.Abs(b.y - a.y);

            return (diffX <= 1 && diffY <= 1) && !(diffX == 0 && diffY == 0);
        }

        public static List<int> GetNeighbourIndexList(int currentNode)
        {
            List<int> neighbourIndexes = new List<int>();

            Point p = GetPoint(currentNode);
            int x = p.x;
            int y = p.y;

            for (int i = -1; i < 2; i++)
            {
                for (int j = -1; j < 2; j++)
                {
                    if (!(i==0 && j==0) && BoundaryCheck(x+j,y+i))
                    {
                        neighbourIndexes.Add(GetIndex(x + j, y + i));              
                    }
                }
            }
            return neighbourIndexes;
        }

        public static int[] GetNeighbourIndexListFast(int currentNode)
        {
            int[] neighbourIndexes = new int[8];

            Point p = GetPoint(currentNode);
            int x = p.x;
            int y = p.y;

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
        
        public static double ToRadians(double angle)
        {
            return (Math.PI / 180) * angle;
        }

        public static Point CalculateAveragePoint(List<int> indexes)
        {
            int sumX = 0;
            int sumY = 0;
            int count = indexes.Count;

            for (int i = 0; i < count; i++)
            {
                Point p = GetPoint(indexes[i]);
                sumX += p.x;
                sumY += p.y;
            }
            return new Point(sumX/count,sumY/count);
        }

        public static int FromWorldToFrameCoordinates(CameraSpacePoint cp)
        {
            int xPixelCoordinateOfPoint = (int)Math.Round(((cp.X * 1000) / GlobVar.MaxHorizontalWidth) * (GlobVar.ScaledFrameWidth / 2) + (GlobVar.ScaledFrameWidth / 2));
            int yPixelCoordinateOfPoint = (int)Math.Round(((-cp.Y * 1000) / GlobVar.MaxVerticalHeight) * (GlobVar.ScaledFrameHeight / 2) + (GlobVar.ScaledFrameHeight / 2));

            return GetIndex(xPixelCoordinateOfPoint, yPixelCoordinateOfPoint);

        }

        public static CameraSpacePoint CalculateMeanPoint(List<int> points)
        {

            float sumX = 0;
            float sumY = 0;
            float sumZ = 0;

            int count = points.Count;

            foreach (var index in points)
            {
                CameraSpacePoint p = GlobVar.MedianFilteredPointCloud[index];
                sumX += p.X;
                sumY += p.Y;
                sumZ += p.Z;
            }

            return new CameraSpacePoint()
            {
                X = sumX/count,
                Y = sumY/count,
                Z = sumZ/count
            };
        }

        public static float DistanceClosestCandidate(int indexPoint)
        {
            float minDistance = float.MaxValue;

            foreach (var head in GlobVar.ValidatedCandidateHeads)
            {
                var headIndex = head.CenterPointIndex;
                float currentDistance = GetEuclideanDistance(headIndex, indexPoint);
                if (currentDistance < minDistance)
                {
                    minDistance = currentDistance;
                }
            }
            return minDistance;
        }

        public static double CalculateFPS(TimeSpan currentTime, TimeSpan lastTime)
        {
            double deltaTime = currentTime.Subtract(lastTime).TotalMilliseconds;
            return 1000/deltaTime;
        }
    }
}