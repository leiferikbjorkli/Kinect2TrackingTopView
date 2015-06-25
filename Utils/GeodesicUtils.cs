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
    internal static class GeodesicUtils
    {
        //anatomical landmarks are defined as pixels at constant differences
        //from top of head

        private static void AddTorso(Body body, List<int> torsoPoints )
        {

            CameraSpacePoint currentMean = GlobUtils.CalculateMeanPoint(torsoPoints);
            if (BodyUtils.HasTorsoTracking(body.Id))
            {
                CameraSpacePoint meanLastTenFrames = BodyUtils.GetAverageTorsoLocationLastTenFrames(body.Id);

                if (GlobUtils.GetEuclideanDistance(meanLastTenFrames, currentMean) < Thresholds.LastTenFramesTorsoMaxDistance)
                {
                    Torso torso = new Torso(currentMean);
                    body.AddTorso(torso);

                    //Graphics.DrawPoint(currentMean);
                }
            }
            else
            {
                Torso torso = new Torso(currentMean);
                body.AddTorso(torso);
                //Graphics.DrawPoint(currentMean);
            }

        }

        private static void AddHands(Body body, List<int> handPoints )
        {
            List<CameraSpacePoint> handCenterPoints = GroupHandPointsDisjointSet(handPoints, body);

            Hand[] newHands = new Hand[2];

            var averageHandLocationsLastTenFrames = BodyUtils.GetAverageHandLocationsLastTenFrames(body.Id);
            CameraSpacePoint avgFirstHandPoint = averageHandLocationsLastTenFrames[0];
            CameraSpacePoint avgSecondHandPoint = averageHandLocationsLastTenFrames[1];

            Dictionary<int, float> distances = new Dictionary<int, float>();

            int index = 0;
            foreach (var handCenterPoint in handCenterPoints)
            {
                float distToFirstHandAverage = GlobUtils.GetEuclideanDistance(avgFirstHandPoint, handCenterPoint);
                distances.Add(index, distToFirstHandAverage);
                index++;
                float distToSecondHandAverage = GlobUtils.GetEuclideanDistance(avgSecondHandPoint, handCenterPoint);
                distances.Add(index, distToSecondHandAverage);
                index++;
            }
            var sortedDistances = distances.OrderBy(kvp => kvp.Value);


            if (BodyUtils.HasFirstHandTracking(body.Id) || BodyUtils.HasSecondHandTracking(body.Id))
            {
                bool[] handsUsed = new bool[] { false, false };

                foreach (var distance in sortedDistances)
                {
                    if (distance.Key == 0)
                    {
                        if (BodyUtils.HasFirstHandTracking(body.Id))
                        {
                            if (distances[distance.Key] < Thresholds.LastTenFramesHandMaxDistance)
                            {
                                Hand hand = new Hand(handCenterPoints[0], 0);
                                if (newHands[0] == null && !handsUsed[0])
                                {
                                    newHands[0] = hand;
                                    handsUsed[0] = true;
                                }
                            }
                        }
                        else
                        {
                            Hand hand = new Hand(handCenterPoints[0], 0);
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
                            if (distances[distance.Key] < Thresholds.LastTenFramesHandMaxDistance)
                            {
                                Hand hand = new Hand(handCenterPoints[0], 1);
                                if (newHands[1] == null && !handsUsed[0])
                                {
                                    newHands[1] = hand;
                                    handsUsed[0] = true;
                                }
                            }
                        }
                        else
                        {
                            Hand hand = new Hand(handCenterPoints[0], 1);
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
                            if (distances[distance.Key] < Thresholds.LastTenFramesHandMaxDistance)
                            {
                                Hand hand = new Hand(handCenterPoints[1], 0);
                                if (newHands[0] == null && !handsUsed[1])
                                {
                                    newHands[0] = hand;
                                    handsUsed[1] = true;
                                }
                            }
                        }
                        else
                        {
                            Hand hand = new Hand(handCenterPoints[1], 0);
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
                            if (distances[distance.Key] < Thresholds.LastTenFramesHandMaxDistance)
                            {
                                Hand hand = new Hand(handCenterPoints[1], 1);
                                if (newHands[1] == null && !handsUsed[1])
                                {
                                    newHands[1] = hand;
                                    handsUsed[1] = true;
                                }
                            }
                        }
                        else
                        {
                            Hand hand = new Hand(handCenterPoints[1], 1);
                            if (newHands[1] == null && !handsUsed[1])
                            {
                                newHands[1] = hand;
                                handsUsed[1] = true;
                            }
                        }
                    }
                }

                if (!float.IsNaN(avgFirstHandPoint.Z))
                {
                    Graphics.DrawPoint(avgFirstHandPoint);
                }
                if (!float.IsNaN(avgSecondHandPoint.Z))
                {
                    Graphics.DrawPoint(avgSecondHandPoint);
                }
            }
            else
            {
                for (int i = 0; i < handCenterPoints.Count; i++)
                {
                    newHands[i] = new Hand(handCenterPoints[i], i);
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

            var results = dijkstra.Perform(head.CenterPointIndex, GlobVar.AdjacancyList);

            var handPoints = new List<int>();
            var torsoPoints = new List<int>();
            
            foreach (var result in results.MinimumDistance)
            {
                if (result.Value > 0.0f && result.Value < 0.2f)
                {
                    //GlobVar.Canvas[result.Key] = 255;

                }
                else if (result.Value < 0.5f && result.Value > 0.2f)
                {
                    //GlobVar.Canvas[result.Key] = 200;
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
            Dictionary<int,Dictionary<int,float>> adjacancyList = new Dictionary<int, Dictionary<int, float>>();

            for (int i = 0; i < GlobVar.ScaledFrameLength; i++)
            {
                if (pointCloud[i].Z != GlobVar.MaxDepthMeter)
                {
                    //check if distance to candidate is close enough
                    List<int> neighbourList = GlobUtils.GetNeighbourIndexList(i);
                    Dictionary<int,float> neighbourNodes = new Dictionary<int, float>();
                    foreach (var neighbour in neighbourList)
                    {
                        if (pointCloud[neighbour].Z != GlobVar.MaxDepthMeter)
                        {
                            float dist = GlobUtils.GetEuclideanDistance(neighbour, i);
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
                handPoints.Add(GlobVar.MedianFilteredPointCloud[handPointsIndexes[i]]);
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
