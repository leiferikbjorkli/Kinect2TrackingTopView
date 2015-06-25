//
// Written by Leif Erik Bjoerkli
//


using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;

namespace InteractionDetection
{
    static class ClassificationUtils
    {

        public static int GetHighestPointInSurroundingArea(Point candidate)
        {
            const int searchPixelRange = 10;

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
                    if (pixelIndex != -1)
                    {
                        float currentHeight = GlobVar.MedianFilteredPointCloud[pixelIndex].Z;
                        if (currentHeight < maxHeight)
                        {
                            maxHeight = currentHeight;
                            maxPointIndex = pixelIndex;
                        } 
                    }
                }
            }

            return maxPointIndex;
        }

        public static List<int> ConnectedComponentLabelingIterative(int startIndex,int maxDepth)
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
                CameraSpacePoint currentPoint = GlobVar.MedianFilteredPointCloud[currentIndex];


                int[] neighbours = GlobUtils.GetNeighbourIndexListFast(currentIndex);

                for (int i = 0; i < neighbours.Length; i++)
                {
                    int neighbourIndex = neighbours[i];
                    if (neighbourIndex == -1)
                    {
                        continue;
                    }
                    float neighbourDepth = GlobVar.MedianFilteredPointCloud[neighbourIndex].Z;
                    CameraSpacePoint neighbourPoint = GlobVar.MedianFilteredPointCloud[neighbourIndex];

                    if (!headPixels.Contains(neighbourIndex) &&
                        Thresholds.ConnectedComponentLabelingMaxDistanceBetweenPoints > GlobUtils.GetEuclideanDistance(currentPoint,neighbourPoint))
                    {
                        Q.Enqueue(neighbourIndex);
                        headPixels.Add(neighbourIndex);
                    }
                }
                maxDepth--;
            }
            return headPixels;
            
        }
        
    }
}
