//
// Written by Leif Erik Bjoerkli
//


using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Documents;
using Microsoft.Kinect;

namespace InteractionDetection
{
    static class GeodesicUtils
    {
        //anatomical landmarks are defined as pixels at constant differences
        //from top of head

        private static void AddTorso(Body body, List<int> torsoPoints )
        {

            CameraSpacePoint currentAvg = BodyUtils.CalculateCenterPointFromValidPoints(torsoPoints);
            if (BodyUtils.HasTorsoTracking(body.Id))
            {
                CameraSpacePoint avgLastFrames = BodyUtils.GetAverageTorsoLocationLastFrames(body.Id);

                //Console.WriteLine(GlobUtils.GetEuclideanDistance(avgLastFiveFrames, currentAvg));
                if (GlobUtils.GetEuclideanDistance(avgLastFrames, currentAvg) < Thresholds.LastFramesTorsoMaxDistance)
                {
                    Torso torso = new Torso(currentAvg, avgLastFrames);
                    body.AddTorso(torso);

                    //Graphics.DrawPoint(currentMean);
                }
            }
            else
            {
                Torso torso = new Torso(currentAvg, new CameraSpacePoint() { X = float.NaN, Y = float.NaN, Z = float.NaN });
                body.AddTorso(torso);
                //Graphics.DrawPoint(currentMean);
            }

        }

        private static void AddHands(Body body, List<int> handPoints )
        {
            List<CameraSpacePoint> handCenterPoints = GroupHandPointsDisjointSet(handPoints, body);

            Hand[] newHands = new Hand[2];

            var averageHandLocationsLastFiveFrames = BodyUtils.GetAverageHandLocationsLastFrames(body.Id);
            CameraSpacePoint avgFirstHandPoint = averageHandLocationsLastFiveFrames[0];
            CameraSpacePoint avgSecondHandPoint = averageHandLocationsLastFiveFrames[1];

            Dictionary<int, float> distancesToHandAverages = new Dictionary<int, float>();

            int index = 0;
            foreach (var handCenterPoint in handCenterPoints)
            {
                float distToFirstHandAverage = GlobUtils.GetEuclideanDistance(avgFirstHandPoint, handCenterPoint);
                distancesToHandAverages.Add(index, distToFirstHandAverage);
                index++;
                float distToSecondHandAverage = GlobUtils.GetEuclideanDistance(avgSecondHandPoint, handCenterPoint);
                distancesToHandAverages.Add(index, distToSecondHandAverage);
                index++;
            }
            var sortedDistancesToHandAverages = distancesToHandAverages.OrderBy(kvp => kvp.Value);


            if (BodyUtils.HasFirstHandTracking(body.Id) || BodyUtils.HasSecondHandTracking(body.Id))
            {
                bool[] handsUsed = new bool[] { false, false };

                foreach (var distance in sortedDistancesToHandAverages)
                {
                    if (distance.Key == 0)
                    {
                        if (BodyUtils.HasFirstHandTracking(body.Id))
                        {
                            if (distancesToHandAverages[distance.Key] < Thresholds.LastFramesHandMaxDistance)
                            {
                                Hand hand = new Hand(handCenterPoints[0], 0, avgFirstHandPoint);
                                if (newHands[0] == null && !handsUsed[0])
                                {
                                    newHands[0] = hand;
                                    handsUsed[0] = true;
                                }
                            }
                        }
                        else
                        {
                            Hand hand = new Hand(handCenterPoints[0], 0, avgFirstHandPoint);
                            if (newHands[0] == null && !handsUsed[0])
                            {
                                newHands[0] = hand;
                                handsUsed[0] = true;
                            }
                        }
                    }
                    else if (distance.Key == 1)
                    {
                        if (BodyUtils.HasSecondHandTracking(body.Id))
                        {
                            if (distancesToHandAverages[distance.Key] < Thresholds.LastFramesHandMaxDistance)
                            {
                                Hand hand = new Hand(handCenterPoints[0], 1, avgSecondHandPoint);
                                if (newHands[1] == null && !handsUsed[0])
                                {
                                    newHands[1] = hand;
                                    handsUsed[0] = true;
                                }
                            }
                        }
                        else
                        {
                            Hand hand = new Hand(handCenterPoints[0], 1, avgSecondHandPoint);
                            if (newHands[1] == null && !handsUsed[0])
                            {
                                newHands[1] = hand;
                                handsUsed[0] = true;
                            }
                        }
                    }
                    else if (distance.Key == 2)
                    {
                        if (BodyUtils.HasFirstHandTracking(body.Id))
                        {
                            if (distancesToHandAverages[distance.Key] < Thresholds.LastFramesHandMaxDistance)
                            {
                                Hand hand = new Hand(handCenterPoints[1], 0, avgFirstHandPoint);
                                if (newHands[0] == null && !handsUsed[1])
                                {
                                    newHands[0] = hand;
                                    handsUsed[1] = true;
                                }
                            }
                        }
                        else
                        {
                            Hand hand = new Hand(handCenterPoints[1], 0, avgFirstHandPoint);
                            if (newHands[0] == null && !handsUsed[1])
                            {
                                newHands[0] = hand;
                                handsUsed[1] = true;
                            }
                        }
                    }
                    else if (distance.Key == 3)
                    {
                        if (BodyUtils.HasSecondHandTracking(body.Id))
                        {
                            if (distancesToHandAverages[distance.Key] < Thresholds.LastFramesHandMaxDistance)
                            {
                                Hand hand = new Hand(handCenterPoints[1], 1, avgSecondHandPoint);
                                if (newHands[1] == null && !handsUsed[1])
                                {
                                    newHands[1] = hand;
                                    handsUsed[1] = true;
                                }
                            }
                        }
                        else
                        {
                            Hand hand = new Hand(handCenterPoints[1], 1, avgSecondHandPoint);
                            if (newHands[1] == null && !handsUsed[1])
                            {
                                newHands[1] = hand;
                                handsUsed[1] = true;
                            }
                        }
                    }
                }

                //if (!float.IsNaN(avgFirstHandPoint.Z) &&
                //Diagnostics.ShowHandAvgPoint) {
                //    GraphicsUtils.DrawPoint(avgFirstHandPoint);
                //}
                //if (!float.IsNaN(avgSecondHandPoint.Z) && Diagnostics.ShowHandAvgPoint)
                //{
                //    GraphicsUtils.DrawPoint(avgSecondHandPoint);
                //}
            }
            else
            {
                for (int i = 0; i < handCenterPoints.Count; i++)
                {
                    newHands[i] = new Hand(handCenterPoints[i], i,new CameraSpacePoint(){X = float.NaN,Y = float.NaN,Z = float.NaN});
                }
            }

            body.AddHands(newHands);

            //foreach (var hand in body.Hands) {
            //    if (hand != null)
            //    {
            //        Graphics.DrawPoint(hand.CenterPoint);
            //    }
            //}

        }

        public static void AddBodyLandmarks(Head head,Body body)
        {
            // Calculate shortest paths
            var dijkstra = new DijkstraFast(GlobVar.AdjacancyList.Count);

            //var v = GlobVar.SubtractedFilteredPointCloud[head.CenterPointIndex];

            int startIndex = head.HighestPointIndex;

            if (!GlobVar.AdjacancyList.ContainsKey(startIndex))
            {
                startIndex = GlobUtils.GetClosestValidNeighbourInAdjacencyList(startIndex);
            }

            if (startIndex == -1)
            {
                return;
            }

            var results = dijkstra.Perform(startIndex, GlobVar.AdjacancyList);

            var handPoints = new List<int>();
            var torsoPoints = new List<int>();
            
            foreach (var result in results.MinimumDistance)
            {
                if (result.Value < 1.1f)
                {
                    GlobVar.HeatCanvas[result.Key] = body.Id;
                }
                if (result.Value > 0.0f && result.Value < 0.2f)
                {
                    //GlobVar.Canvas[result.Key] = 255;

                }
                else if (result.Value < 0.8f && result.Value > 0.3f)
                {
                    if (Diagnostics.ShowTorso)
                    {
                        GlobVar.Canvas[result.Key] = 200;
                    }
                    torsoPoints.Add(result.Key);

                }
                else if (result.Value < 1.1f && result.Value > 0.9f)
                {
                    if (Diagnostics.ShowHandPixels)
                    {
                        GlobVar.Canvas[result.Key] = 150;
                    }
                    handPoints.Add(result.Key);
                }
            }

            if (torsoPoints.Count > Thresholds.TorsoPointMinCount)
            {
                AddTorso(body,torsoPoints);
            }


            if (handPoints.Count > Thresholds.HandPointMinCount)
            {
                AddHands(body, handPoints);
            }

        }

        public static Dictionary<int, Dictionary<int, float>> CreateGeodesicGraph(CameraSpacePoint[] pointCloud)
        {
            var adjacancyList = new Dictionary<int, Dictionary<int, float>>();

            var maxDepth = GlobVar.MaxDepthMeter;

            for (int i = 0; i < GlobVar.ScaledFrameLength; i++)
            {
                if (pointCloud[i].Z != maxDepth && !float.IsInfinity(pointCloud[i].X) && !float.IsInfinity(pointCloud[i].Y))
                {
                    int[] neighbourList = GlobUtils.GetNeighbourIndexListFast(i);
                    Dictionary<int,float> neighbourNodes = new Dictionary<int, float>();
                    foreach (var neighbour in neighbourList)
                    {
                        if (pointCloud[neighbour].Z != maxDepth && neighbour != -1)
                        {
                            float dist = GlobUtils.GetEuclideanDistance(neighbour, i);

                            //check if distance to candidate is close enough

                            if (dist < Thresholds.GeodesicGraphMaxDistanceBetweenPoints && GlobUtils.DistanceClosestCandidate(neighbour) < Thresholds.GeodesicGraphMaxDistanceToCandidates)
                            {
                                neighbourNodes.Add(neighbour,dist);   
                            }
                        }
                    }
                    if (neighbourNodes.Count > 0)
                    {
                        adjacancyList.Add(i,neighbourNodes);
                    }
                }
            }

            return adjacancyList;
        }

        private static List<CameraSpacePoint> GroupHandPointsDisjointSet(List<int> handPointsIndexes, Body body)
        {
            // Adapt threshold to extreme points

            List<CameraSpacePoint> handPoints = new List<CameraSpacePoint>();
            for (int i = 0; i < handPointsIndexes.Count; i++)
            {
                handPoints.Add(GlobVar.SubtractedFilteredPointCloud[handPointsIndexes[i]]);
            }

            DisjointSet3D disjointSetHandPoints = new DisjointSet3D(handPoints);
            for (int i = 0; i < handPoints.Count; i++)
            {
                for (int j = 0; j < handPoints.Count; j++)
                {
                    if (i == j) { continue; };
                    //Console.WriteLine(GlobUtils.GetDistanceInFrame(handPoints[i], handPoints[j]));
                    if (GlobUtils.GetEuclideanDistance(handPoints[i], handPoints[j]) < Thresholds.HandPointGroupingMaxDistance)
                    {
                        disjointSetHandPoints.Union(i, j);
                    }
                }
            }

            List<CameraSpacePoint> handCenterPoints = FilterDisjointSet3D(disjointSetHandPoints);

            return handCenterPoints;

        }

        private static List<CameraSpacePoint> FilterDisjointSet3D(DisjointSet3D disjointSet)
        {
            Dictionary<int, int> filteredCandidatesIndex = new Dictionary<int, int>();

            for (int i = 0; i < disjointSet.Count; i++)
            {
                int setRepresentative = disjointSet.Find(i);
                int setSize = disjointSet.SetSize(setRepresentative);
                if (!filteredCandidatesIndex.ContainsKey(setRepresentative) && setSize > Thresholds.HandPointMinCount)
                {
                    filteredCandidatesIndex.Add(setRepresentative, setSize);
                }
            }

            var sortedFilteredCandidatesIndex = filteredCandidatesIndex.OrderByDescending(kvp => kvp.Value);

            List<CameraSpacePoint> filteredTopCandidates = new List<CameraSpacePoint>();

            int counter = 0;
            foreach (var filteredCandidate in sortedFilteredCandidatesIndex)
            {
                var cp = new CameraSpacePoint
                {
                    X = disjointSet.AverageX[filteredCandidate.Key],
                    Y = disjointSet.AverageY[filteredCandidate.Key],
                    Z = disjointSet.AverageZ[filteredCandidate.Key]
                };
                filteredTopCandidates.Add(cp);

                counter++;
                if (counter == 2)
                {
                    break;
                }
            }

            return filteredTopCandidates;
        }
    }
}
