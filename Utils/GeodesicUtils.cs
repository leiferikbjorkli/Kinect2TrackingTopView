using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Documents;
using Microsoft.Kinect;

namespace InteractionDetection
{
    internal static class GeodesicUtils
    {
        //anatomical landmarks are defined as pixels at constant differences
        //from top of head



        public static void CalculateShortestPaths(Head head,Body body)
        {
            // pass search are
            var dijkstra = new DijkstraFast(GlobVar.AdjacancyList.Count);

            int headCenter = head.CenterPoint;

            if(!GlobVar.AdjacancyList.ContainsKey(headCenter))
            {
                return;
            }

            var results = dijkstra.Perform(head.CenterPoint, GlobVar.AdjacancyList);

            //foreach (var pair in results.MinimumPath)
            //{
            //    GlobVar.Canvas[pair.Key] = 255;
            //}

            var handPoints = new List<int>();
            var shoulderPoints = new List<int>();


            foreach (var result in results.MinimumDistance)
            {
                if (result.Value > 0.0f && result.Value < 0.2f)
                {
                    GlobVar.Canvas[result.Key] = 255;

                }
                else if (result.Value < 0.5f && result.Value > 0.2f)
                {
                    GlobVar.Canvas[result.Key] = 200;
                    shoulderPoints.Add(result.Key);

                }
                else if (result.Value < 1.1f && result.Value > 0.9f)
                {
                    GlobVar.Canvas[result.Key] = 255;
                    handPoints.Add(result.Key);
                }

                // Assume person is looking towards the middle
            }

            if (handPoints.Count < 50 || shoulderPoints.Count <= 10)
            {
                if (GlobVar.Bodies.Count > 0)
                {
                    Body lastBody = BodyUtils.GetLastFrameBody(body);
                    if (lastBody != null)
                    {
                        body.AddLeftShoulder(lastBody.LeftShoulder);
                        body.AddRightShoulder(lastBody.RightShoulder);
                    }

                }
            }
            else
            {
                Point avgHandPoint = GlobUtils.CalculateAveragePoint(handPoints);

                BodyUtils.GroupShoulderPoints(shoulderPoints, head.CenterPoint, avgHandPoint, body);

                if (body.LeftShoulder != null && body.RightShoulder != null)
                {
                    BodyUtils.GroupHandPoints(head.CenterPoint, handPoints, results.MinimumPath, body);
                }
            }

            if (body.LeftShoulder == null || body.RightShoulder == null)
            {
                
            }
        }

        public static Dictionary<int, Dictionary<int, float>> CreateGeodesicGraph(CameraSpacePoint[] pointCloud)
        {
            Dictionary<int,Dictionary<int,float>> adjacancyList = new Dictionary<int, Dictionary<int, float>>();

            for (int i = 0; i < GlobVar.ScaledFrameLength; i++)
            {
                if (pointCloud[i].Z != GlobVar.MaxDepthMeter)
                {
                    List<int> neighbourList = GlobUtils.GetNeighbourIndexList(i);
                    Dictionary<int,float> neighbourNodes = new Dictionary<int, float>();
                    foreach (var neighbour in neighbourList)
                    {
                        if (pointCloud[neighbour].Z != GlobVar.MaxDepthMeter)
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
