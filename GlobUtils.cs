using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;

namespace InteractionDetection
{
    static class GlobUtils
    {

        static public int GetIndex(int x, int y)
        {
            if (x < 0 || x > 255 || y < 0 || y > 211)
            {
                 throw new System.ArgumentException("Coordinates are incorrect");
            }

            return y * GlobVar.scaledFrameWidth + x;
        }

        static public int GetIndex(Point p)
        {
            int x = p.x;
            int y = p.y;
            if (x < 0 || x > 255 || y < 0 || y > 211)
            {
                throw new System.ArgumentException("Coordinates are incorrect");
            }

            return y * GlobVar.scaledFrameWidth + x;
        }

        static public Point GetPoint(int index)
        {
            if (index < 0 || index > GlobVar.scaledFrameLength-1)
            {
                throw new System.ArgumentException("Index out of bounds");
            }
            Point p;
            p.x = index % GlobVar.scaledFrameWidth;
            double yDouble = index / GlobVar.scaledFrameWidth;
            int y = (int)Math.Floor(yDouble);
            p.y = y;
            return p;
        }

        static public int CalculatePixelAreaFromIndexes(int a, int b, int c)
        {
            return (GetPoint(b).x + 1 - GetPoint(a).x) * (GetPoint(c).y + 1 - GetPoint(a).y);
        }

        static public float GetEuclideanDistance(int indexA, int indexB) 
        {
            CameraSpacePoint a = GlobVar.scaledCameraSpacePoints[indexA];
            CameraSpacePoint b = GlobVar.scaledCameraSpacePoints[indexB];

            return (float) Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y) + (a.Z - b.Z) * (a.Y - b.Y));
        }

        static public float GetEuclideanDistance(Point a, Point b)
        {
            CameraSpacePoint aCameraSpace = GlobVar.scaledCameraSpacePoints[GetIndex(a)];
            CameraSpacePoint bCameraSpace = GlobVar.scaledCameraSpacePoints[GetIndex(b)];

            return (float)
                    Math.Sqrt((aCameraSpace.X - bCameraSpace.X)*(aCameraSpace.X - bCameraSpace.X) +
                              (aCameraSpace.Y - bCameraSpace.Y)*(aCameraSpace.Y - bCameraSpace.Y) +
                              (aCameraSpace.Z - bCameraSpace.Z)*(aCameraSpace.Z - bCameraSpace.Z));
        }
        static public int GetRectangleCenterPointIndex(IndexRectangle rect)
        {
            var a = GetPoint(rect.a);
            var b = GetPoint(rect.b);
            var c = GetPoint(rect.c);

            return GetIndex(a.x + (b.x - a.x) / 2, a.y + (c.y - a.y) / 2);
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

            return new Point((a.x + b.x) / 2, (a.y + b.y) / 2);
        }

        static public Point GetRectangleCenterPoint(ThreePointRectangle rect)
        {
            return new Point(rect.a.x + (rect.b.x - rect.a.x) / 2, rect.a.y + (rect.c.y - rect.a.y) / 2);
        }
        static public float GetDistanceInFrame(Point a, Point b)
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
        public static int AdjustXBoundaries(int x)
        {
            if (x < 0) { x = 0; }
            else if (x > 255) { x = 255; }
            return x;
        }

        public static int AdjustYBoundaries(int y)
        {
            if (y < 0) { y = 0; }
            else if (y > 255) { y = 255; }
            return y;
        }

        public static Point AdjustBoundaries(Point p)
        {
            int x = p.x;
            int y = p.y;

            if (x < 0){x = 0;}
            else if (x > 255){x = 255;}
            else if (y<0){y = 0;}
            else if (y>211){y = 211;}
            else{return p;}
            return new Point(x,y);
        }
    }
}
