using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace InteractionDetection
{
    static class ClassificationUtils
    {

        public static int GetHighestPointInSurroundingArea(Point candidate)
        {
            const int searchPixelRange = 20;

            int xStartIndex = candidate.x - searchPixelRange;
            int yStartIndex = candidate.y - searchPixelRange;
            int xStopIndex = candidate.x + searchPixelRange;
            int yStopIndex = candidate.y + searchPixelRange;

            int maxPointIndex = GlobUtils.GetIndex(candidate);
            float maxHeight = GlobVar.MedianFilteredPointCloud[maxPointIndex].Z;

            for (int i = yStartIndex; i < yStopIndex; i++)
            {
                for (int j = xStartIndex; j < xStopIndex; j++)
                {
                    int pixelIndex = GlobUtils.GetIndex(j, i);
                    float currentHeight = GlobVar.MedianFilteredPointCloud[pixelIndex].Z;
                    if (currentHeight < maxHeight && pixelIndex != -1)
                    {
                        maxHeight = currentHeight;
                        maxPointIndex = pixelIndex;
                    }
                }
            }

            return maxPointIndex;
        }

        public static int ConnectedComponentLabelingIterative(int startIndex,int maxDepth,Head head)
        {
            var Q = new Queue<int>();

            Q.Enqueue(startIndex);

            List<int> headPixels = new List<int>();

            headPixels.Add(startIndex);
            while (Q.Count > 0)
            {
                if (maxDepth<1)
                {
                    break;
                }
                int currentIndex = Q.Dequeue();

                float currentDepth = GlobVar.MedianFilteredPointCloud[currentIndex].Z;
                int[] neighbours = GlobUtils.GetNeighbourIndexListFast(currentIndex);

                for (int i = 0; i < neighbours.Length; i++)
                {
                    int neighbourIndex = neighbours[i];
                    if (neighbourIndex == -1)
                    {
                        continue;
                    }
                    float neighbourDepth = GlobVar.MedianFilteredPointCloud[neighbourIndex].Z;

                    if (!headPixels.Contains(neighbourIndex) &&
                        Thresholds.Labeling > (Math.Abs(neighbourDepth - currentDepth)))
                    {
                        Q.Enqueue(neighbourIndex);
                        headPixels.Add(neighbourIndex);
                    }
                }
                maxDepth--;
            }

            if (headPixels.Count > 10)
            {
                head.AddHeadPixels(headPixels);
                return 1;
            }
            return -1;
        }
        
        //private static float FindHighest(Point currentPoint,int searchDepth,ref Point highestPoint,float highestValue)
        //{
        //    int x = currentPoint.x;
        //    int y = currentPoint.y;
        //    float currentValue = GlobVar.depthMap[GlobUtils.GetIndex(x,y)];

        //    if (currentValue>highestValue)
        //    {
        //        highestPoint = currentPoint;
        //        highestValue = currentValue;
        //        if (searchDepth <= 0)
        //        {
        //            return currentValue;
        //        }
        //        searchDepth--;
        //        highestValue = FindHighest(new Point(x + 1, y), searchDepth, ref highestPoint, highestValue);
        //        highestValue = FindHighest(new Point(x - 1, y), searchDepth, ref highestPoint, highestValue);
        //        highestValue = FindHighest(new Point(x, y + 1), searchDepth, ref highestPoint, highestValue);
        //        highestValue = FindHighest(new Point(x, y - 1), searchDepth, ref highestPoint, highestValue);
        //        return highestValue;
        //    }
        //    return highestValue;
        //}

        //public static void ConnectedComponentLabeling(Point pixel, byte label,float previousDepth, int recursionLimit)
        //{
        //    if(pixel.x > GlobVar.scaledFrameWidth-1 || pixel.x < 0 || pixel.y > GlobVar.scaledFrameHeight-1 || pixel.y < 0)
        //    {
        //        return;
        //    }

        //    int pixelIndex = GlobUtils.GetIndex(pixel);
        //    float pixelDepth = GlobVar.depthMap[pixelIndex];

        //    int x = pixel.x;
        //    int y = pixel.y;

        //    if (GlobVar.Canvas[pixelIndex] == label || Thresholds.Labeling < (Math.Abs(pixelDepth - previousDepth)))
        //        return;
        //    GlobVar.Canvas[pixelIndex] = label;
        //    recursionLimit--;
        //    if (recursionLimit <= 1)
        //    {
        //        return;
                
        //    }

        //    ConnectedComponentLabeling(new Point(x - 1, y), label, pixelDepth, recursionLimit);
        //    ConnectedComponentLabeling(new Point(x - 1, y - 1), label, pixelDepth, recursionLimit);
        //    ConnectedComponentLabeling(new Point(x, y - 1), label, pixelDepth, recursionLimit);
        //    ConnectedComponentLabeling(new Point(x + 1, y - 1), label, pixelDepth, recursionLimit);
        //    ConnectedComponentLabeling(new Point(x + 1, y), label, pixelDepth, recursionLimit);
        //    ConnectedComponentLabeling(new Point(x + 1, y + 1), label, pixelDepth, recursionLimit);
        //    ConnectedComponentLabeling(new Point(x, y + 1), label, pixelDepth, recursionLimit);
        //    ConnectedComponentLabeling(new Point(x - 1, y + 1), label, pixelDepth, recursionLimit);
        //}

        //public static Point GetHighestLocalPoint(Point point)
        //{
        //    float highestLocalValue = 0;
        //    Point highestLocalPoint = new Point();
        //    FindHighest(point, 10, ref highestLocalPoint, highestLocalValue);

        //    return highestLocalPoint;

        //}

    }
}
