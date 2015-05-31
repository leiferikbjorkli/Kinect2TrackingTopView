using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;

namespace InteractionDetection
{
    internal static class GlobUtils
    {

        public static int GetIndex(int x, int y)
        {
            if (x < 0 || x > 255 || y < 0 || y > 211)
            {
                return -1;
            }

            return y*GlobVar.scaledFrameWidth + x;
        }

        public static int GetIndexNoBoundaryCheck(int x, int y)
        {
            return y*GlobVar.scaledFrameWidth + x;
        }

        public static int GetIndex(Point p)
        {
            int x = p.x;
            int y = p.y;
            if (x < 0 || x > 255 || y < 0 || y > 211)
            {
                throw new System.ArgumentException("Coordinates are incorrect");
            }

            return y*GlobVar.scaledFrameWidth + x;
        }

        public static Point GetPoint(int index)
        {
            if (index < 0 || index > GlobVar.scaledFrameLength - 1)
            {
                throw new System.ArgumentException("Index out of bounds");
            }
            Point p;
            p.x = index%GlobVar.scaledFrameWidth;
            double yDouble = index/GlobVar.scaledFrameWidth;
            int y = (int) Math.Floor(yDouble);
            p.y = y;
            return p;
        }

        public static int CalculatePixelAreaFromIndexes(int a, int b, int c)
        {
            return (GetPoint(b).x + 1 - GetPoint(a).x)*(GetPoint(c).y + 1 - GetPoint(a).y);
        }

        public static float GetEuclideanDistance(int indexA, int indexB)
        {
            CameraSpacePoint a = GlobVar.InvertedCloud[indexA];
            CameraSpacePoint b = GlobVar.InvertedCloud[indexB];

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
            CameraSpacePoint aCameraSpace = GlobVar.InvertedCloud[GetIndex(a)];
            CameraSpacePoint bCameraSpace = GlobVar.InvertedCloud[GetIndex(b)];

            return (float)
                Math.Sqrt((aCameraSpace.X - bCameraSpace.X)*(aCameraSpace.X - bCameraSpace.X) +
                          (aCameraSpace.Y - bCameraSpace.Y)*(aCameraSpace.Y - bCameraSpace.Y) +
                          (aCameraSpace.Z - bCameraSpace.Z)*(aCameraSpace.Z - bCameraSpace.Z));
        }

        public static int GetRectangleCenterPointIndex(IndexRectangle rect)
        {
            var a = GetPoint(rect.a);
            var b = GetPoint(rect.b);
            var c = GetPoint(rect.c);

            return GetIndex(a.x + (b.x - a.x)/2, a.y + (c.y - a.y)/2);
        }

        public static int GetMidPointIndex(int aIndex, int bIndex)
        {
            var a = GetPoint(aIndex);
            var b = GetPoint(bIndex);

            return GetIndex((a.x + b.x)/2, (a.y + b.y)/2);
        }

        public static Point GetMidPointFromIndexes(int aIndex, int bIndex)
        {
            var a = GetPoint(aIndex);
            var b = GetPoint(bIndex);

            return new Point((a.x + b.x)/2, (a.y + b.y)/2);
        }

        public static Point GetRectangleCenterPoint(ThreePointRectangle rect)
        {
            return new Point(rect.a.x + (rect.b.x - rect.a.x)/2, rect.a.y + (rect.c.y - rect.a.y)/2);
        }

        public static float GetDistanceInFrame(Point a, Point b)
        {
            return (float) Math.Sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
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
            return index > 0 && index < GlobVar.scaledFrameLength;
        }


        public static int AdjustXBoundaries(int x)
        {
            if (x < 0)
            {
                x = 0;
            }
            else if (x > 255)
            {
                x = 255;
            }
            return x;
        }

        public static int AdjustYBoundaries(int y)
        {
            if (y < 0)
            {
                y = 0;
            }
            else if (y > 255)
            {
                y = 255;
            }
            return y;
        }

        public static Point AdjustBoundaries(Point p)
        {
            int x = p.x;
            int y = p.y;

            if (x < 0)
            {
                x = 0;
            }
            else if (x > 255)
            {
                x = 255;
            }
            else if (y < 0)
            {
                y = 0;
            }
            else if (y > 211)
            {
                y = 211;
            }
            else
            {
                return p;
            }
            return new Point(x, y);
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

        public static List<int> GetNeighbourIndexList5x5(int currentNode)
        {
            List<int> neighbourIndexes = new List<int>();

            Point p = GetPoint(currentNode);
            int x = p.x;
            int y = p.y;

            for (int i = -2; i < 3; i++)
            {
                for (int j = -2; j < 3; j++)
                {
                    if (!(i == 0 && j == 0) && BoundaryCheck(x + j, y + i))
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

    }
}