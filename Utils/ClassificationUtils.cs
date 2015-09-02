//
// Copyright (c) Leif Erik Bjoerkli, Norwegian University of Science and Technology, 2015.
// Distributed under the MIT License.
// (See accompanying file LICENSE or copy at http://opensource.org/licenses/MIT)
//  

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;

namespace Kinect2TrackingTopView
{
    static class ClassificationUtils
    {
        /// <summary>
        /// Groups points where the distance between them in the euclidean XY-plane is below threshold. The highest point is kept.
        /// </summary>
        public static List<int> GroupCandidatesHighestPoints(List<int> highestPointIndexes, float groupingMaxDistance)
        {
            var groupedPoints = new List<int>();

            var pointDepths = new Dictionary<int, float>();
            for (int i = 0; i < highestPointIndexes.Count; i++)
            {
                if (!pointDepths.ContainsKey(highestPointIndexes[i]))
                {
                    pointDepths.Add(highestPointIndexes[i], GlobVar.SubtractedFilteredPointCloud[highestPointIndexes[i]].Z);
                }
            }

            var sortedPointDepths = pointDepths.OrderBy(kvp => kvp.Value);

            foreach (var sortedPointDepth in sortedPointDepths)
            {
                int currentIndex = sortedPointDepth.Key;
                bool add = true;

                foreach (var groupedPoint in groupedPoints)
                {
                    if (GlobUtils.GetEuclideanDistanceXYPlane(groupedPoint, currentIndex) < groupingMaxDistance)
                    {
                        add = false;
                    }
                }
                if (add)
                {
                    groupedPoints.Add(sortedPointDepth.Key);
                }
            }

            return groupedPoints;
        }

        /// <summary>
        /// Recursively searches for the highest point connected to the startindex. The search stops when the maximum searchdepth is reached.
        /// </summary>
        /// <param name="currentIndex"></param>
        /// <param name="searchDepth"></param>
        /// <returns></returns>
        public static int GetHighestConnectingPoint(int currentIndex, int searchDepth)
        {
            float currentDepth = GlobVar.SubtractedFilteredPointCloud[currentIndex].Z;

            int[] neighbours = GlobUtils.GetNeighbour5X5IndexList(currentIndex);

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

        /// <summary>
        /// Connected component labeling. Height difference between connected pixels are used as evaluation criteria.
        /// </summary>
        /// <param name="startIndex">start index of the labeling</param>
        /// <param name="maxPixels">maximum amount of pixels allowed in resulting labeled component</param>
        /// <returns>Returns a list of connected headpoints</returns>
        public static List<int> ConnectedComponentLabeling(int startIndex,int maxPixels)
        {
            var q = new Queue<int>();

            q.Enqueue(startIndex);

            List<int> headPixels = new List<int>();

            headPixels.Add(startIndex);
            while (q.Count > 0)
            {
                if (maxPixels<1)
                {
                    break;
                }
                int currentIndex = q.Dequeue();

                CameraSpacePoint currentPoint = GlobVar.SubtractedFilteredPointCloud[currentIndex];

                int[] neighbours = GlobUtils.GetNeighbour3X3IndexList(currentIndex);

                for (int i = 0; i < neighbours.Length; i++)
                {
                    int neighbourIndex = neighbours[i];
                    if (neighbourIndex == -1)
                    {
                        continue;
                    }
                    CameraSpacePoint neighbourPoint = GlobVar.SubtractedFilteredPointCloud[neighbourIndex];

                    if (!headPixels.Contains(neighbourIndex) &&
                        Thresholds.ClassificationLabelingMaxHeightBetweenPoints > GlobUtils.GetHeightDifference(currentPoint,neighbourPoint))
                    {
                        q.Enqueue(neighbourIndex);
                        headPixels.Add(neighbourIndex);
                        maxPixels--;
                    }
                }
                
            }
            return headPixels;
        }
    }
}
