//
// Copyright (c) Leif Erik Bjoerkli, Norwegian University of Science and Technology, 2015.
// Distributed under the MIT License.
// (See accompanying file LICENSE or copy at http://opensource.org/licenses/MIT)
//  

using System.Collections.Generic;
using Microsoft.Kinect;

namespace Kinect2TrackingTopView
{
    internal static class GeodesicUtils
    {
        public static Dictionary<int, Dictionary<int, float>> GeodesicGraph { get; private set; }

        /// <summary>
        ///     Creates a geodesic graph of the foreground. Points in cameraspace that are closer than a threshold are connected in the graph.
        ///     Also, the points need to be closer than threshold to the already validated candidate head points.
        /// </summary>
        public static void CreateGeodesicGraph(CameraSpacePoint[] pointCloud)
        {
            GeodesicGraph = new Dictionary<int, Dictionary<int, float>>();
            const float maxDepth = GlobVar.MaxSensingDepth;

            for (var i = 0; i < GlobVar.ScaledFrameLength; i++)
            {
                if (pointCloud[i].Z != maxDepth && !float.IsInfinity(pointCloud[i].X) &&
                    !float.IsInfinity(pointCloud[i].Y))
                {
                    var neighbourList = GlobUtils.GetNeighbour3X3IndexList(i);
                    var neighbourNodes = new Dictionary<int, float>();
                    foreach (var neighbour in neighbourList)
                    {
                        if (neighbour != -1)
                        {
                            if (pointCloud[neighbour].Z != maxDepth)
                            {
                                var dist = GlobUtils.GetEuclideanDistance(neighbour, i);

                                if (dist < Thresholds.GeodesicGraphMaxDistanceBetweenPoints &&
                                    BodyUtils.GetDistanceToClosestHeadCandidate(neighbour) <
                                    Thresholds.GeodesicGraphMaxDistanceToCandidates)
                                {
                                    neighbourNodes.Add(neighbour, dist);
                                }
                            }
                        }
                    }
                    if (neighbourNodes.Count > 0)
                    {
                        GeodesicGraph.Add(i, neighbourNodes);
                    }
                }
            }
            if (Logger.ShowGeodesicGraph)
            {
                foreach (var v in GeodesicGraph)
                {
                    GlobVar.GraphicsCanvas[v.Key] = 255;
                }
            }
        }

        ///<remarks>
        /// Due to imprecisions in the coordinate mapping, if the startIndex for the search in the geodesic graph doesn't exist, the closest neighbour is used. A breadth first search is used.
        /// </remarks>
        public static int GetClosestValidNeighbourInGeodesicGraph(int startIndex)
        {
            var q = new Queue<int>();

            q.Enqueue(startIndex);

            var maxPixels = 30;
            while (q.Count > 0)
            {
                if (maxPixels < 1)
                {
                    return -1;
                }
                var currentIndex = q.Dequeue();

                var neighbours = GlobUtils.GetNeighbour3X3IndexList(currentIndex);

                for (var i = 0; i < neighbours.Length; i++)
                {
                    var neighbourIndex = neighbours[i];
                    if (neighbourIndex == -1)
                    {
                        continue;
                    }

                    if (GeodesicGraph.ContainsKey(neighbourIndex))
                    {
                        if (GeodesicGraph[neighbourIndex].Count > 1)
                        {
                            return neighbourIndex;
                        }
                    }
                    q.Enqueue(neighbourIndex);
                    maxPixels--;
                }
            }
            return -1;
        }
    }
}