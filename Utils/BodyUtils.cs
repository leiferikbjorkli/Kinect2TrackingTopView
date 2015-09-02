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
using System.Windows;
using Microsoft.Kinect;

namespace Kinect2TrackingTopView
{
    static class BodyUtils
    {

        // Number of frames to consider when calculating average values.
        private const int NumberOfLastFrames = 5;

        private const int NumberOfTrackedFrames = 10;
        private const int NumberOfTrackedFramesStartIndex = NumberOfTrackedFrames - NumberOfLastFrames;

        /// <summary>
        /// Calculates the average point in camera space from the input points. Invalid pixels are discarded.
        /// </summary>
        public static CameraSpacePoint CalculateAveragePointFromValidPoints(List<int> pointIndexes)
        {
            var points = new List<CameraSpacePoint>();

            for (var i = 0; i < pointIndexes.Count; i++)
            {
                CameraSpacePoint rawPoint = GlobVar.SubtractedFilteredPointCloud[pointIndexes[i]];

                if (rawPoint.Z != GlobVar.MaxSensingDepth)
                {
                    points.Add(rawPoint);
                }
            }

            float x = 0;
            float y = 0;
            float z = 0;

            var xValuesCount = 0;
            var yValuesCount = 0;

            for (var i = 0; i < points.Count; i++)
            {
                var point = points[i];
                if (!float.IsInfinity(point.X))
                {
                    x += point.X;
                    xValuesCount++;
                }
                if (!float.IsInfinity(point.Y))
                {
                    y += point.Y;
                    yValuesCount++;
                }
                z += points[i].Z;
            }

            x = x / xValuesCount;
            y = y / yValuesCount;
            z = z / points.Count;

            var p = new CameraSpacePoint
            {
                X = x,
                Y = y,
                Z = z
            };

            return p;
        }

        ///  <summary>
        ///     Uses Dijkstra to find the shortest geodesic path from the top of the head to the rest of the body. The body parts
        ///     torso and hands are classified as the points from the geodesic map that fall into bins with a certain distance from
        ///     the top of the head.
        /// </summary>
        /// <returns>Returns a dictionary with the heads as keys and labels as values. If head isn't identified, the label is -1</returns>
        public static void AddBodyRegions(Dictionary<int, Dictionary<int, float>> geodesicGraph, Head head, Body body)
        {
            var startIndex = head.HighestPointIndex;

            if (!geodesicGraph.ContainsKey(startIndex))
            {
                startIndex = GeodesicUtils.GetClosestValidNeighbourInGeodesicGraph(startIndex);
            }

            if (startIndex == -1)
            {
                return;
            }

            var minDistances = Dijkstra.Perform(startIndex, geodesicGraph);

            var handPoints = new List<int>();
            var torsoPoints = new List<int>();

            foreach (var minDistance in minDistances)
            {
                if (minDistance.Value < 1.1f)
                {
                    // Include body in current heatcanvas
                    GlobVar.HeatCanvas[minDistance.Key] = body.Id;
                }
                if (minDistance.Value < 0.2f)
                {
                    if (Logger.ShowHeadRegionPixels)
                    {
                        GlobVar.GraphicsCanvas[minDistance.Key] = 255;
                    }
                    torsoPoints.Add(minDistance.Key);
                }
                else if (minDistance.Value < 0.6f && minDistance.Value > 0.2f)
                {
                    if (Logger.ShowTorsoPixels)
                    {
                        GlobVar.GraphicsCanvas[minDistance.Key] = 200;
                    }
                    torsoPoints.Add(minDistance.Key);
                }
                else if (minDistance.Value < 1.2f && minDistance.Value > 0.9f)
                {
                    if (Logger.ShowHandPixels)
                    {
                        GlobVar.GraphicsCanvas[minDistance.Key] = 150;
                    }
                    handPoints.Add(minDistance.Key);
                }
            }

            if (torsoPoints.Count > Thresholds.TorsoPointMinCount)
            {
                body.AddTorso(torsoPoints);
            }

            if (handPoints.Count > Thresholds.HandPointMinCount)
            {
                body.AddHands(handPoints);
            }
        }

        /// <summary>
        /// Groups the hand points if closer than threshold.
        /// </summary>
        public static List<CameraSpacePoint> GroupHandPoints(List<int> handPointsIndexes)
        {
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
                    if (GlobUtils.GetEuclideanDistance(handPoints[i], handPoints[j]) < Thresholds.HandPointGroupingMaxDistance)
                    {
                        disjointSetHandPoints.Union(i, j);
                    }
                }
            }

            List<CameraSpacePoint> handCenterPoints = FilterHandCandidates(disjointSetHandPoints);

            return handCenterPoints;

        }

        /// <summary>
        /// Filters the hand candidate points. Only the two candidate points belonging to the two biggest sets are kept.
        /// </summary>
        private static List<CameraSpacePoint> FilterHandCandidates(DisjointSet3D groupedSet)
        {
            Dictionary<int, int> filteredCandidatesIndex = new Dictionary<int, int>();

            for (int i = 0; i < groupedSet.Count; i++)
            {
                int setRepresentative = groupedSet.Find(i);
                int setSize = groupedSet.SetSize(setRepresentative);
                if (!filteredCandidatesIndex.ContainsKey(setRepresentative) && setSize > Thresholds.HandPointMinCount)
                {
                    filteredCandidatesIndex.Add(setRepresentative, setSize);
                }
            }

            var sortedFilteredCandidatesIndex = filteredCandidatesIndex.OrderByDescending(kvp => kvp.Value);

            var filteredTopCandidates = new List<CameraSpacePoint>();

            int counter = 0;
            foreach (var filteredCandidate in sortedFilteredCandidatesIndex)
            {
                var cp = new CameraSpacePoint
                {
                    X = groupedSet.AverageX[filteredCandidate.Key],
                    Y = groupedSet.AverageY[filteredCandidate.Key],
                    Z = groupedSet.AverageZ[filteredCandidate.Key]
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

        /// <summary>
        /// Find id of new heads based on euclidean distance to heads detected in previous frame.
        /// </summary>
        /// <returns>Returns a dictionary with the heads as keys and labels as values. If head isn't identified, the label is -1</returns>
        public static Dictionary<Head,int> IdentifyHeads(List<Head> validatedCandidateHeads)
        {
            var identifiedHeads = new Dictionary<Head, int>();

            var recentBodyIds = GetRecentBodyIds();

            var avgLocationsRecentHeads = GetAvgLocationsRecentHeadsFromIds(recentBodyIds);

            var distancesFromCandidateHeadsToRecentHeads = GetDistancesFromCandidateHeadsToRecentHeads(validatedCandidateHeads, avgLocationsRecentHeads);

            var sortedDistances = distancesFromCandidateHeadsToRecentHeads.OrderBy(i => i.Value);

            foreach (var sortedDistance in sortedDistances)
            {
                var candidatePair = sortedDistance.Key;
                var candidateHead = candidatePair.Item1;
                var headId = candidatePair.Item2;
                if (!identifiedHeads.ContainsKey(candidateHead) && !identifiedHeads.ContainsValue(headId))
                {
                    identifiedHeads.Add(candidateHead,headId);
                }
            }

            foreach (var candidateHead in validatedCandidateHeads)
            {
                if (!identifiedHeads.ContainsKey(candidateHead))
                {
                    identifiedHeads.Add(candidateHead,-1);
                }
            }

            return identifiedHeads;
        }

        private static Dictionary<Tuple<Head, int>, float> GetDistancesFromCandidateHeadsToRecentHeads(List<Head> validatedCandidateHeads,
            Dictionary<int, CameraSpacePoint> avgLocationsRecentHeads)
        {
            var distancesFromCandidateHeadsToRecentHeads = new Dictionary<Tuple<Head, int>, float>();

            foreach (var candidateHead in validatedCandidateHeads)
            {
                foreach (var avgLocationRecentHead in avgLocationsRecentHeads)
                {
                    var assignment = new Tuple<Head, int>(candidateHead, avgLocationRecentHead.Key);
                    var distanceToCandidate = GlobUtils.GetEuclideanDistance(candidateHead.CenterPoint,
                        avgLocationRecentHead.Value);

                    if (distanceToCandidate < Thresholds.LastFramesHeadMaxDistance)
                    {
                        distancesFromCandidateHeadsToRecentHeads.Add(assignment, distanceToCandidate);
                    }
                }
            }
            return distancesFromCandidateHeadsToRecentHeads;
        }

        private static Dictionary<int, CameraSpacePoint> GetAvgLocationsRecentHeadsFromIds(List<int> recentBodyIds)
        {
            var avgLocationsRecentHeads = new Dictionary<int, CameraSpacePoint>();

            foreach (var bodyId in recentBodyIds)
            {
                avgLocationsRecentHeads.Add(bodyId, GetAverageHeadLocationLastTenFrames(bodyId));
                if (Logger.ShowHeadAvgCenterLastTenFrames)
                {
                    if (!float.IsNaN(GetAverageHeadLocationLastTenFrames(bodyId).Z))
                    {
                        GraphicsUtils.DrawPoint(GetAverageHeadLocationLastTenFrames(bodyId));
                    }
                }
            }
            return avgLocationsRecentHeads;
        }

        public static Dictionary<int, float> GetDistancesToHandAveragesLastFrames(List<CameraSpacePoint> handCenterPoints, CameraSpacePoint avgFirstHandPoint,
            CameraSpacePoint avgSecondHandPoint)
        {
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
            return distancesToHandAverages;
        }

        private static List<int> GetRecentBodyIds()
        {
            List<int> recentBodyIds = new List<int>();
            for (int i = 0; i < BodiesHistory.Get.Count; i++)
            {
                foreach (var body in BodiesHistory.Get.ElementAt(i))
                {
                    if (!recentBodyIds.Contains(body.Id))
                    {
                        recentBodyIds.Add(body.Id);
                    }
                }
            }
            return recentBodyIds;
        }

        private static CameraSpacePoint GetAverageHeadLocationLastTenFrames(int newBodyId)
        {
            float sumX = 0;
            float sumY = 0;
            float sumZ = 0;

            float bodiesCount = 0;

            for (int i = 0; i < BodiesHistory.Get.Count; i++)
            {
                var bodies = BodiesHistory.Get.ElementAt(i);
                for (int j = 0; j < bodies.Count; j++)
                {
                    var body = bodies[j];
                    if (body.Id == newBodyId)
                    {
                        CameraSpacePoint p = body.Head.CenterPoint;
                        sumX += p.X;
                        sumY += p.Y;
                        sumZ += p.Z;
                        bodiesCount ++;
                    }
                }
            }
            return new CameraSpacePoint
            {
                X = sumX / bodiesCount,
                Y = sumY / bodiesCount,
                Z = sumZ / bodiesCount
            };
        }

        ///<remarks>
        /// Only returns a valid average point if tracking is complete for the specified last frames.
        /// </remarks>
        public static CameraSpacePoint GetAverageHeadLocationLastFrames(int newBodyId)
        {
            float sumX = 0;
            float sumY = 0;
            float sumZ = 0;

            float bodiesCount = 0;

            for (int i = NumberOfTrackedFramesStartIndex; i < BodiesHistory.Get.Count; i++)
            {
                var bodies = BodiesHistory.Get.ElementAt(i);
                for (int j = 0; j < bodies.Count; j++)
                {
                    var body = bodies[j];
                    if (body.Id == newBodyId)
                    {
                        CameraSpacePoint p = body.Head.CenterPoint;
                        sumX += p.X;
                        sumY += p.Y;
                        sumZ += p.Z;
                        bodiesCount++;
                    }
                }
            }
            if (bodiesCount == NumberOfLastFrames)
            {
                return new CameraSpacePoint
                {
                    X = sumX / bodiesCount,
                    Y = sumY / bodiesCount,
                    Z = sumZ / bodiesCount
                };
            }
            return new CameraSpacePoint
            {
                X = float.NaN,
                Y = float.NaN,
                Z = float.NaN
            };
        }

        ///<remarks>
        /// Only returns a valid average point if tracking is complete for the specified last frames.
        /// </remarks>
        public static CameraSpacePoint GetAverageTorsoLocationLastFrames(int newBodyId)
        {
            float sumX = 0;
            float sumY = 0;
            float sumZ = 0;

            int bodiesCount = 0;

            for (int i = NumberOfTrackedFramesStartIndex; i < BodiesHistory.Get.Count; i++)
            {
                var bodies = BodiesHistory.Get.ElementAt(i);
                for (int j = 0; j < bodies.Count; j++)
                {
                    var body = bodies[j];
                    if (body.Id == newBodyId && body.Torso != null)
                    {
                        CameraSpacePoint p = body.Torso.CenterPoint;
                        sumX += p.X;
                        sumY += p.Y;
                        sumZ += p.Z;
                        bodiesCount++;
                    }
                }
            }

            if (bodiesCount == NumberOfLastFrames)
            {
                return new CameraSpacePoint
                {
                    X = sumX / bodiesCount,
                    Y = sumY / bodiesCount,
                    Z = sumZ / bodiesCount
                };
            }
            return new CameraSpacePoint
            {
                X = float.NaN,
                Y = float.NaN,
                Z = float.NaN
            };
        }

        public static CameraSpacePoint[] GetAverageHandLocationsLastFrames(int bodyId)
        {
            float sumXfirstHand = 0;
            float sumYfirstHand = 0;
            float sumZfirstHand = 0;

            float sumXsecondHand = 0;
            float sumYsecondHand = 0;
            float sumZsecondHand = 0;

            float firstHandCount = 0;
            float secondHandCount = 0;


            for (int i = NumberOfTrackedFramesStartIndex; i < BodiesHistory.Get.Count; i++)
            {
                var bodies = BodiesHistory.Get.ElementAt(i);
                for (int j = 0; j < bodies.Count; j++)
                {
                    var body = bodies[j];
                    if (body.Id == bodyId)
                    {
                        foreach (var hand in body.Hands)
                        {
                            if (hand != null)
                            {
                                CameraSpacePoint p = hand.CenterPoint;
                                if (hand.Id == 0)
                                {
                                    sumXfirstHand += p.X;
                                    sumYfirstHand += p.Y;
                                    sumZfirstHand += p.Z;
                                    firstHandCount++;
                                }
                                else if (hand.Id == 1)
                                {
                                    sumXsecondHand += p.X;
                                    sumYsecondHand += p.Y;
                                    sumZsecondHand += p.Z;
                                    secondHandCount++;
                                }
                            }
                        }

                    }
                }
            }

            var firstHand = new CameraSpacePoint
            {
                X = sumXfirstHand / firstHandCount,
                Y = sumYfirstHand / firstHandCount,
                Z = sumZfirstHand / firstHandCount
            };

            var secondHand = new CameraSpacePoint
            {
                X = sumXsecondHand / secondHandCount,
                Y = sumYsecondHand / secondHandCount,
                Z = sumZsecondHand / secondHandCount
            };

            return new CameraSpacePoint[] { firstHand, secondHand };
        }

        public static bool HasFirstHandTracking(int bodyId)
        {
            int trackedHands = 0;
            for (int i = NumberOfTrackedFramesStartIndex; i < BodiesHistory.Get.Count; i++)
            {
                foreach (var body in BodiesHistory.Get.ElementAt(i))
                {
                    if (body.Id == bodyId && body.Hands[0] != null)
                    {
                        trackedHands++;
                    }
                }
            }

            return trackedHands == NumberOfLastFrames;
        }

        public static bool HasSecondHandTracking(int bodyId)
        {
            int trackedHands = 0;
            for (int i = NumberOfTrackedFramesStartIndex; i < BodiesHistory.Get.Count; i++)
            {
                foreach (var body in BodiesHistory.Get.ElementAt(i))
                {
                    if (body.Id == bodyId && body.Hands[1] != null)
                    {
                        trackedHands++;
                    }
                }
            }

            return trackedHands == NumberOfLastFrames;
        }

        public static bool HasTorsoTracking(int bodyId)
        {
            int trackedTorsos = 0;
            for (int i = NumberOfTrackedFramesStartIndex; i < BodiesHistory.Get.Count; i++)
            {
                foreach (var body in BodiesHistory.Get.ElementAt(i))
                {
                    if (body.Id == bodyId && body.Torso != null)
                    {
                        trackedTorsos++;
                    }
                }
            }

            return trackedTorsos == NumberOfLastFrames;
        }

        public static bool HasBodyTracking()
        {
            return BodiesHistory.Get.Count > 0;
        }

        /// <summary>
        /// Returns the bodies from history that has the specified id.
        /// </summary>
        public static List<Body> GetBodiesWithIdFromHistory(int bodyId)
        {
            List<Body> bodiesWithId = new List<Body>();

            for (int i = 7; i < BodiesHistory.Get.Count-1; i++)
            {
                foreach (var body in BodiesHistory.Get.ElementAt(i))
                {
                    if (body.Id == bodyId)
                    {
                        bodiesWithId.Add(body);
                    }
                }
            }

            return bodiesWithId;
        }

        public static float GetDistanceToClosestHeadCandidate(int indexPoint)
        {
            float minDistance = float.MaxValue;

            foreach (var head in GlobVar.ValidatedCandidateHeads)
            {
                var headIndex = head.HighestPointIndex;
                float currentDistance = GlobUtils.GetEuclideanDistance(headIndex, indexPoint);

                if (currentDistance < minDistance)
                {
                    minDistance = currentDistance;
                }
            }
            return minDistance;
        }

    }


}
