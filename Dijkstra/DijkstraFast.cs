//
// Written by Leif Erik Bjoerkli
//


using System;
using System.Collections;
using System.Diagnostics;
using System.Collections.Generic;
using System.Text;

namespace InteractionDetection
{
    /// <summary> 
    /// Implements a generalized Dijkstra's algorithm to calculate 
    /// both minimum distance and minimum path. 
    /// </summary> 
    /// <remarks> 
    /// For this algorithm, all nodes should be provided, and handled 
    /// in the delegate methods, including the start and finish nodes. 
    /// </remarks> 
    public class DijkstraFast
    {

        /// <summary> 
        /// Creates an instance of the <see cref="Dijkstra"/> class. 
        /// </summary> 
        /// <param name="totalNodeCount"> 
        /// The total number of nodes in the graph. 
        /// </param> 
       
        public DijkstraFast(int totalNodeCount)
        {
            if (totalNodeCount < 3) throw new ArgumentOutOfRangeException("totalNodeCount", totalNodeCount, "Expected a minimum of 3.");
            TotalNodeCount = totalNodeCount;
        }

        protected readonly int TotalNodeCount;

        public struct Results
        {
            /// <summary> 
            /// Prepares a Dijkstra results package. 
            /// </summary> 
            /// <param name="minimumPath"> 
            /// The minimum path array, where each array element index corresponds  
            /// to a node designation, and the array element value is a pointer to 
            /// the node that should be used to travel to this one. 
            /// </param> 
            /// <param name="minimumDistance"> 
            /// The minimum distance from the starting node to the given node. 
            /// </param> 
            public Results(Dictionary<int,int> minimumPath, Dictionary<int,float> minimumDistance)
            {
                MinimumDistance = minimumDistance;
                MinimumPath = minimumPath;
            }

            /// The minimum path array, where each array element index corresponds  
            /// to a node designation, and the array element value is a pointer to 
            /// the node that should be used to travel to this one. 
            public readonly Dictionary<int, int> MinimumPath;

            /// The minimum distance from the starting node to the given node. 
            public readonly Dictionary<int, float> MinimumDistance;
        }

        public class QueueElement : IComparable
        {
            public int index;
            public float weight;

            public QueueElement() { }
            public QueueElement(int i, float val)
            {
                index = i;
                weight = val;
            }

            public int CompareTo(object obj)
            {
                QueueElement outer = (QueueElement)obj;

                if (this.weight > outer.weight)
                    return 1;
                else if (this.weight < outer.weight)
                    return -1;
                else return 0;
            } 
        }


        // start: The node to use as a starting location. 
        // A struct containing both the minimum distance and minimum path 
        // to every node from the given <paramref name="start"/> node. 
        public virtual Results Perform(int start, Dictionary<int, Dictionary<int, float>> adjacancyList)
        {
            // Initialize the distance to every node from the starting node.
 
            // use same as median filter



            Dictionary<int, float> distances = GetStartingTraversalCost(start,adjacancyList);

            // Initialize best path to every node as from the starting node. 

            Dictionary<int, int> path = GetStartingBestPath(start, adjacancyList);

            BasicHeap Q = new BasicHeap();

            foreach (var distance in distances)
            {
                Q.Push(distance.Key,distance.Value);
            }

            while (Q.Count != 0)
            {
                int v = Q.Pop();

                Dictionary<int, float> neighbours = adjacancyList[v];

                foreach (var neigbourIndex in neighbours.Keys)
                {
                    float cost = neighbours[neigbourIndex];


                    if (distances.ContainsKey(neigbourIndex))
                    {
                        if (cost < float.MaxValue && distances[v] + cost < distances[neigbourIndex]) // don't let wrap-around negatives slip by 
                        {
                            // We have found a better way to get at relative 
                            distances[neigbourIndex] = distances[v] + cost; // record new distance 
                            path[neigbourIndex] = v;

                            Q.Push(neigbourIndex, distances[neigbourIndex]);
                        } 
                    }
                }
            }

            return new Results(path, distances);
        }

        // Uses the Dijkstra algorithhm to find the minimum path from one node
        // to another.  Return a struct containing both the minimum distance and
        // minimum path to every node from the given start node.

        //public virtual int[] GetMinimumPath(int start, int finish) {
        //    if (start < finish)
        //    {
        //        int tmp = start;
        //        start = finish;
        //        finish = tmp;
        //    }

        //    Results results = Perform(start);
        //    return GetMinimumPath(start, finish, results.MinimumPath);
        //}

        // Finds an array of nodes that provide the shortest path 
        // from one given node to another. 
        // ShortestPath : P array of the completed algorithm:
        // The list of nodes that provide the one step at a time path from 
        protected virtual int[] GetMinimumPath(int start, int finish, Dictionary<int,int> shortestPath)
        {
            Stack<int> path = new Stack<int>();

            do
            {
                path.Push(finish);
                finish = shortestPath[finish]; // step back one step toward the start point 
            }
            while (finish != start);
            return path.ToArray();
        }

        // Initializes the P array for the algorithm. 
        // A fresh P array will set every single node's source node to be  
        // the starting node, including the starting node itself. 
        protected virtual Dictionary<int, int> GetStartingBestPath(int startingNode, Dictionary<int, Dictionary<int, float>> adjacancyList)
        {
            Dictionary<int,int> p = new Dictionary<int, int>();

            foreach (var node in adjacancyList)
            {
                p.Add(node.Key,startingNode);
            }

            return p;
        }


        protected virtual Dictionary<int, float> GetStartingTraversalCost(int start, Dictionary<int, Dictionary<int, float>> adjacancyList)
        {
            Dictionary<int,float> subset = new Dictionary<int, float>();

            foreach (var node in adjacancyList)
            {
                subset.Add(node.Key,float.MaxValue);
            }
            subset[start] = 0;

            Dictionary<int, float> neighbours = adjacancyList[start];
            foreach (var nearby in neighbours.Keys)
            {
                subset[nearby] = neighbours[nearby];
            }

            return subset; 
        }

    }
}