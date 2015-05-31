using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Documents;
using InteractionDetection.Dijkstra;

namespace InteractionDetection
{
    internal static class GeodesicUtils
    {
        //anatomical landmarks are defined as pixels at constant differences
        //from top of head



        public static void CalculateShortestPaths(Head head)
        {
            // pass search area
            var dijkstra = new DijkstraFast(GlobVar.AdjacancyList.Count);

            var results = dijkstra.Perform(head.CenterPoint, GlobVar.AdjacancyList);

            foreach (var result in results.MinimumDistance)
            {
                if (result.Value > 0.0f && result.Value < 0.2f)
                {
                    GlobVar.canvas[result.Key] = 200;

                }
                else if (result.Value < 0.9f && result.Value > 0.2f)
                {
                    GlobVar.canvas[result.Key] = 100;
                }
                else if (result.Value < 1.1f)
                {
                    GlobVar.canvas[result.Key] = 255;
                }
            }
        }

        //public static Dictionary<int, float[]> CreateGeodesicGraph() {
        //    Dictionary<int, float[]> adjacencyList = new Dictionary<int, float[]>();

        //    for (int i = 1; i < GlobVar.scaledFrameHeight - 1; i += 1)
        //    {
        //        for (int j = 0; j < GlobVar.scaledFrameWidth - 1; j += 1)
        //        {
        //            int currentPointIndex = GlobUtils.GetIndex(j, i);
        //            float[] edges = new float[4];
        //            bool existEdges = false;

        //            int rightUpPointIndex = GlobUtils.GetIndex(j + 1, i - 1);
        //            float dist = GlobUtils.GetEuclideanDistance(currentPointIndex, rightUpPointIndex);
        //            if (dist < Thresholds.GeodesicGraph && dist != 0)
        //            {
        //                edges[0] = dist;
        //                existEdges = true;
        //            }

        //            int rightPixelIndex = GlobUtils.GetIndex(j + 1, i);
        //            dist = GlobUtils.GetEuclideanDistance(currentPointIndex, rightPixelIndex);
        //            if (dist < Thresholds.GeodesicGraph && dist != 0)
        //            {
        //                edges[1] = dist;
        //                existEdges = true;
        //            }

        //            int rightDownPointIndex = GlobUtils.GetIndex(j + 1, i + 1);
        //            dist = GlobUtils.GetEuclideanDistance(currentPointIndex, rightDownPointIndex);
        //            if (dist < Thresholds.GeodesicGraph && dist != 0)
        //            {
        //                edges[2] = dist;
        //                existEdges = true;
        //            }

        //            int downPixelIndex = GlobUtils.GetIndex(j, i + 1);
        //            dist = GlobUtils.GetEuclideanDistance(currentPointIndex, downPixelIndex);
        //            if (dist < Thresholds.GeodesicGraph && dist != 0)
        //            {
        //                edges[3] = dist;
        //                existEdges = true;
        //            }

        //            if (existEdges)
        //            {
        //                adjacencyList.Add(currentPointIndex, edges);
        //            }

        //        }
        //    }

        //    return adjacencyList;


        //}

        //public static void CreateGeodesicGraph() {
        //    List<Edge> edges = new List<Edge>();

        //    Dictionary<int,Node> nodes = new Dictionary<int,Node>();

        //    for (int i = 1; i < GlobVar.scaledFrameHeight-1; i+=1)
        //    {
        //        for (int j = 0; j < GlobVar.scaledFrameWidth-1; j+=1)
        //        {
        //            int currentPointIndex = GlobUtils.GetIndex(j, i);

        //            int rightUpPointIndex = GlobUtils.GetIndex(j + 1, i - 1);
        //            float dist = GlobUtils.GetEuclideanDistance(currentPointIndex, rightUpPointIndex);
        //            if (dist < Thresholds.GeodesicGraph){edges.Add(new Edge(dist,currentPointIndex,rightUpPointIndex));}

        //            int rightPixelIndex = GlobUtils.GetIndex(j + 1, i);
        //            dist = GlobUtils.GetEuclideanDistance(currentPointIndex, rightPixelIndex);
        //            if (dist < Thresholds.GeodesicGraph){edges.Add(new Edge(dist,currentPointIndex,rightPixelIndex));}

        //            int rightDownPointIndex = GlobUtils.GetIndex(j + 1, i + 1);
        //            dist = GlobUtils.GetEuclideanDistance(currentPointIndex, rightDownPointIndex);
        //            if (dist < Thresholds.GeodesicGraph){edges.Add(new Edge(dist,currentPointIndex,rightDownPointIndex));}

        //            int downPixelIndex = GlobUtils.GetIndex(j, i + 1);
        //            dist = GlobUtils.GetEuclideanDistance(currentPointIndex, downPixelIndex);
        //            if (dist < Thresholds.GeodesicGraph){edges.Add(new Edge(dist,currentPointIndex,downPixelIndex));}

        //        }
        //    }

        //}

        public static Dictionary<int, Dictionary<int, float>> CreateGeodesicGraph()
        {
            Dictionary<int,Dictionary<int,float>> adjacancyList = new Dictionary<int, Dictionary<int, float>>();

            for (int i = 0; i < GlobVar.scaledFrameLength; i++)
            {
                if (GlobVar.depthMap[i] != 0)
                {
                    List<int> neighbourList = GlobUtils.GetNeighbourIndexList(i);
                    Dictionary<int,float> neighbourNodes = new Dictionary<int, float>();
                    foreach (var neighbour in neighbourList)
                    {
                        if (GlobVar.depthMap[neighbour] != 0)
                        {
                            float dist = GlobUtils.GetEuclideanDistance(neighbour, i);
                            if (dist<Thresholds.GeodesicGraph)
                            {
                                neighbourNodes.Add(neighbour,dist);   
                            }
                        }
                    }

                    adjacancyList.Add(i,neighbourNodes);
                }
            }

            return adjacancyList;
        }
    }
}
