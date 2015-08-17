//
// Copyright (c) Leif Erik Bjoerkli, Norwegian University of Science and Technology, 2015.
// Distributed under the MIT License.
// (See accompanying file LICENSE or copy at http://opensource.org/licenses/MIT)
//  

//
// Inspired by Tolga Birdals implementation in
// http://www.codeproject.com/Articles/24816/A-Fast-Priority-Queue-Implementation-of-the-Dijkst
//

using System;
using System.Collections;
using System.Diagnostics;
using System.Collections.Generic;
using System.Text;

namespace Kinect2TrackingTopView
{
    public static class Dijkstra
    {
        /// <summary>
        /// Performs the dijkstra shortest path search on the provided geodesic graph. 
        /// </summary>
        /// <returns>Returns a dictionary with the minimum distances from the starting node to the given nodes</returns>
        public static Dictionary<int, float> Perform(int start, Dictionary<int, Dictionary<int, float>> geodesicGraph)
        {
            Dictionary<int, float> distances = GetStartingTraversalCost(start,geodesicGraph);

            BasicHeap q = new BasicHeap();

            foreach (var distance in distances)
            {
                q.Push(distance.Key,distance.Value);
            }

            while (q.Count != 0)
            {
                int v = q.Pop();

                Dictionary<int, float> neighbours = geodesicGraph[v];

                foreach (var neigbourIndex in neighbours.Keys)
                {
                    float cost = neighbours[neigbourIndex];

                    if (distances.ContainsKey(neigbourIndex))
                    {
                        if (cost < float.MaxValue && distances[v] + cost < distances[neigbourIndex]) 
                        {
                            distances[neigbourIndex] = distances[v] + cost;

                            q.Push(neigbourIndex, distances[neigbourIndex]);
                        } 
                    }
                }
            }

            return distances;
        }

        /// <summary>
        /// Initializes the traversal costs to neighbour nodes of start node. The rest of the nodes are initialized to float.maxvalue.
        /// </summary>
        private static Dictionary<int, float> GetStartingTraversalCost(int start, Dictionary<int, Dictionary<int, float>> geodesicGraph)
        {
            var subset = new Dictionary<int, float>();

            foreach (var node in geodesicGraph)
            {
                subset.Add(node.Key,float.MaxValue);
            }
            subset[start] = 0;

            Dictionary<int, float> neighbours = geodesicGraph[start];
            foreach (var neighbour in neighbours.Keys)
            {
                subset[neighbour] = neighbours[neighbour];
            }

            return subset; 
        }
    }
}