//
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

        public static float GetEuclideanDistance(Point a, Point b)
        {
            CameraSpacePoint aCameraSpace = GlobVar.SubtractedFilteredPointCloud[GetIndex(a)];
            CameraSpacePoint bCameraSpace = GlobVar.SubtractedFilteredPointCloud[GetIndex(b)];

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

        public static float GetEuclideanDistanceXYPlane(CameraSpacePoint a, CameraSpacePoint b)
        {

            float distance =
                (float)Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y));

            if (float.IsNaN(distance))
            {
                return float.MaxValue;
            }
            return distance;
        }

        public static float GetEuclideanDistanceXYPlane(int aIndex, int bIndex)
        {
            var a = GlobVar.SubtractedFilteredPointCloud[aIndex];
            var b = GlobVar.SubtractedFilteredPointCloud[bIndex];

            float distance =
                (float)Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y));

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

        public static float GetHeightDifference(CameraSpacePoint a, CameraSpacePoint b)
        {
            return Math.Abs(a.Z - b.Z);
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

        public static int[] GetNeighbour5x5IndexList(int currentNode)
        {
            int[] neighbourIndexes = new int[24];

            var frameWidth = GlobVar.ScaledFrameWidth;

            Point p = GetPoint(currentNode);
            int x = p.x;
            int y = p.y;

            
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

        //public static Point CalculateAveragePoint(List<int> indexes)
        //{
        //    int sumX = 0;
        //    int sumY = 0;
        //    int count = indexes.Count;

        //    for (int i = 0; i < count; i++)
        //    {
        //        Point p = GetPoint(indexes[i]);
        //        sumX += p.x;
        //        sumY += p.y;
        //    }
        //    return new Point(sumX/count,sumY/count);
        //}

        public static int FromWorldToFrameCoordinates(CameraSpacePoint cp)
        {
            int xPixelCoordinateOfPoint = (int)Math.Round(((cp.X * 1000) / GlobVar.HalfMaxHorizontalWidth) * (GlobVar.ScaledFrameWidth / 2) + (GlobVar.ScaledFrameWidth / 2));
            int yPixelCoordinateOfPoint = (int)Math.Round(((-cp.Y * 1000) / GlobVar.HalfMaxVerticalHeight) * (GlobVar.ScaledFrameHeight / 2) + (GlobVar.ScaledFrameHeight / 2));

            return GetIndex(xPixelCoordinateOfPoint, yPixelCoordinateOfPoint);

        }

        //public static CameraSpacePoint CalculateAveragePoint(List<int> points)
        //{

        //    float sumX = 0;
        //    float sumY = 0;
        //    float sumZ = 0;

        //    int count = points.Count;

        //    foreach (var index in points)
        //    {
        //        CameraSpacePoint p = GlobVar.SubtractedFilteredPointCloud[index];
        //        sumX += p.X;
        //        sumY += p.Y;
        //        sumZ += p.Z;
        //    }

        //    return new CameraSpacePoint()
        //    {
        //        X = sumX/count,
        //        Y = sumY/count,
        //        Z = sumZ/count
        //    };
        //}

        public static float DistanceClosestCandidate(int indexPoint)
        {
            float minDistance = float.MaxValue;

            foreach (var head in GlobVar.ValidatedCandidateHeads)
            {
                var headIndex = head.HighestPointIndex;
                float currentDistance = GetEuclideanDistance(headIndex, indexPoint);

                //Console.WriteLine(currentDistance);
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

        public static CameraSpacePoint GetHighestPointInRectangle(int bIndex, int cIndex)
        {
            Point pB = GetPoint(bIndex);
            Point pC = GetPoint(cIndex);
            int rectangleWidth = pB.x - pC.x;

            CameraSpacePoint highestPoint = new CameraSpacePoint()
            {
                X = float.PositiveInfinity,
                Y = float.PositiveInfinity,
                Z = float.PositiveInfinity
            };

            for (int i = 0; i < rectangleWidth; i++)
            {
                for (int j = 0; j < rectangleWidth; j++)
                {
                    int currentIndex = GetIndex(pC.x + j, pB.y + i);
                    if (BoundaryCheck(currentIndex))
                    {
                        CameraSpacePoint currentPoint = GlobVar.SubtractedFilteredPointCloud[currentIndex];

                        if (currentPoint.Z < highestPoint.Z)
                        {
                            highestPoint = currentPoint;
                        } 
                    }
                }
            }
            return highestPoint;
        }

        public static CameraSpacePoint GetHighestValidPointInRectangle(int bIndex, int cIndex)
        {
            Point pB = GetPoint(bIndex);
            Point pC = GetPoint(cIndex);
            int rectangleWidth = pB.x - pC.x;

            CameraSpacePoint highestPoint = new CameraSpacePoint()
            {
                X = float.PositiveInfinity,
                Y = float.PositiveInfinity,
                Z = float.PositiveInfinity
            };

            for (int i = 0; i < rectangleWidth; i++)
            {
                for (int j = 0; j < rectangleWidth; j++)
                {
                    int currentIndex = GetIndex(pC.x + j, pB.y + i);
                    if (BoundaryCheck(currentIndex))
                    {
                        CameraSpacePoint currentPoint = GlobVar.SubtractedFilteredPointCloud[currentIndex];

                        if (currentPoint.Z < highestPoint.Z && !float.IsInfinity(currentPoint.X) && !float.IsInfinity(currentPoint.Y))
                        {
                            highestPoint = currentPoint;
                        }
                    }
                }
            }
            return highestPoint;
        }

        public static int GetHighestValidPointIndexInRectangle(int bIndex, int cIndex)
        {
            Point pB = GetPoint(bIndex);
            Point pC = GetPoint(cIndex);
            int rectangleWidth = pB.x - pC.x;

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
                    int currentIndex = GetIndex(pC.x + j, pB.y + i);
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

            if (GlobVar.SubtractedFilteredPointCloud[highestPointIndex].Z == GlobVar.MaxDepthMeter || GlobVar.SubtractedFilteredPointCloud[highestPointIndex].X == 0 || float.IsInfinity(GlobVar.SubtractedFilteredPointCloud[highestPointIndex].X))
            {

            }
            return highestPointIndex;
        }


        public static int GetClosestValidNeighbourInAdjacencyList(int startIndex)
        {
            var Q = new Queue<int>();

            Q.Enqueue(startIndex);

            List<int> headPixels = new List<int>();

            int maxPixels = 30;
            headPixels.Add(startIndex);
            while (Q.Count > 0)
            {
                if (maxPixels < 1)
                {
                    return -1;
                }
                int currentIndex = Q.Dequeue();

                int[] neighbours = GetNeighbourIndexListFast(currentIndex);

                for (int i = 0; i < neighbours.Length; i++)
                {
                    int neighbourIndex = neighbours[i];
                    if (neighbourIndex == -1)
                    {
                        continue;
                    }

                    if (GlobVar.AdjacancyList.ContainsKey(neighbourIndex))
                    {
                        if (GlobVar.AdjacancyList[neighbourIndex].Count > 1)
                        {
                            return neighbourIndex;
                        }
                    }
                    Q.Enqueue(neighbourIndex);
                    headPixels.Add(neighbourIndex);
                    maxPixels--;
                    
                }

            }
            return -1;

        }    
    }


}