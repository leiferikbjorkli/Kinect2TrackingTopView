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
            const int searchPixelRange = Thresholds.ClassificationSearchRangeHighestPoint;

            int xStartIndex = candidate.x - searchPixelRange;
            int yStartIndex = candidate.y - searchPixelRange;
            int xStopIndex = candidate.x + searchPixelRange;
            int yStopIndex = candidate.y + searchPixelRange;

            int maxPointIndex = GlobUtils.GetIndex(candidate);
            float maxHeight = GlobVar.SubtractedFilteredPointCloud[maxPointIndex].Z;

            for (int i = yStartIndex; i < yStopIndex; i++)
            {
                for (int j = xStartIndex; j < xStopIndex; j++)
                {
                    int pixelIndex = GlobUtils.GetIndex(j, i);
                    if (pixelIndex != -1)
                    {
                        float currentHeight = GlobVar.SubtractedFilteredPointCloud[pixelIndex].Z;
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

        public static int GetHighestConnectingPoint(int currentIndex, int searchDepth)
        {
            float currentDepth = GlobVar.SubtractedFilteredPointCloud[currentIndex].Z;

            int[] neighbours = GlobUtils.GetNeighbour5x5IndexList(currentIndex);

            int highestNeighbourIndex = currentIndex;
            float shallowestNeighbourDepth = currentDepth;
            for (int i = 0; i < neighbours.Length; i++)
            {
                int neighbourIndex = neighbours[i];
                if (!GlobUtils.BoundaryCheck(neighbourIndex))
                {
                    continue;
                }
                CameraSpacePoint neighbour = GlobVar.SubtractedFilteredPointCloud[neighbourIndex];

                if (neighbour.Z < shallowestNeighbourDepth && !float.IsInfinity(neighbour.X) && !float.IsInfinity(neighbour.Y))
                {
                    shallowestNeighbourDepth = neighbour.Z;
                    highestNeighbourIndex = neighbourIndex;
                }
            }
            if (highestNeighbourIndex == currentIndex || searchDepth == 0)
            {
                return currentIndex;
            }
            searchDepth--;

            return GetHighestConnectingPoint(highestNeighbourIndex,searchDepth);
        }

        public static List<int> ConnectedComponentLabelingIterative(int startIndex,int maxPixels)
        {
            var Q = new Queue<int>();

            Q.Enqueue(startIndex);

            List<int> headPixels = new List<int>();

            headPixels.Add(startIndex);
            while (Q.Count > 0)
            {
                if (maxPixels<1)
                {
                    break;
                }
                int currentIndex = Q.Dequeue();

                CameraSpacePoint currentPoint = GlobVar.SubtractedFilteredPointCloud[currentIndex];

                int[] neighbours = GlobUtils.GetNeighbourIndexListFast(currentIndex);

                for (int i = 0; i < neighbours.Length; i++)
                {
                    int neighbourIndex = neighbours[i];
                    if (neighbourIndex == -1)
                    {
                        continue;
                    }
                    CameraSpacePoint neighbourPoint = GlobVar.SubtractedFilteredPointCloud[neighbourIndex];

                    //var v = GlobUtils.GetHeightDifference(currentPoint, neighbourPoint);

                    if (!headPixels.Contains(neighbourIndex) &&
                        Thresholds.ClassificationLabelingMaxDistanceBetweenPoints > GlobUtils.GetHeightDifference(currentPoint,neighbourPoint))
                    {
                        Q.Enqueue(neighbourIndex);
                        headPixels.Add(neighbourIndex);
                        maxPixels--;
                    }
                }
                
            }
            return headPixels;
            
        }
        
    }
}
