//
// Written by Leif Erik Bjoerkli
//


using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using Microsoft.Kinect;

namespace InteractionDetection
{
    static class BodyUtils
    {
        private const int NumberOfLastFrames = 5;
        private const int NumberOfTrackedFrames = 10;
        private const int NumberOfTrackedFramesStartIndex = NumberOfTrackedFrames - NumberOfLastFrames;

        public static int CalculateCenterPointHeadPoints(List<int> headPixels )
        {
            List<CameraSpacePoint> headPoints = new List<CameraSpacePoint>();

            for (int i = 0; i < headPixels.Count; i++)
            {
                CameraSpacePoint rawPoint = GlobVar.SubtractedFilteredPointCloud[headPixels[i]];

                if (rawPoint.Z != GlobVar.MaxDepthMeter)
                {
                    headPoints.Add(rawPoint);
                }
            }

            float x = 0;
            float y = 0;
            float z = 0;

            for (int i = 0; i < headPoints.Count; i++)
            {
                x += headPoints[i].X;
                y += headPoints[i].Y;
                z += headPoints[i].Z;
            }

            x = x / headPoints.Count;
            y = y / headPoints.Count;
            z = z / headPoints.Count;


            var p = new CameraSpacePoint
            {
                X = x,
                Y = y,
                Z = z
            };


            int xPixelCoordinateOfPoint = (int)Math.Round(((p.X * 1000) / GlobVar.HalfMaxHorizontalWidth) * (GlobVar.ScaledFrameWidth / 2) + (GlobVar.ScaledFrameWidth / 2));
            int yPixelCoordinateOfPoint = (int)Math.Round(((-p.Y * 1000) / GlobVar.HalfMaxVerticalHeight) * (GlobVar.ScaledFrameHeight / 2) + (GlobVar.ScaledFrameHeight / 2));

            return GlobUtils.GetIndex(xPixelCoordinateOfPoint, yPixelCoordinateOfPoint);
        }

        public static CameraSpacePoint CalculateCenterPointFromValidPoints(List<int> headPixels)
        {
            List<CameraSpacePoint> headPoints = new List<CameraSpacePoint>();

            for (int i = 0; i < headPixels.Count; i++)
            {
                CameraSpacePoint rawPoint = GlobVar.SubtractedFilteredPointCloud[headPixels[i]];

                if (rawPoint.Z != GlobVar.MaxDepthMeter)
                {
                    headPoints.Add(rawPoint);
                }
            }

            float x = 0;
            float y = 0;
            float z = 0;

            int xValuesCount = 0;
            int yValuesCount = 0;

            for (int i = 0; i < headPoints.Count; i++)
            {
                var point = headPoints[i];
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
                z += headPoints[i].Z;
            }

            x = x / xValuesCount;
            y = y / yValuesCount;
            z = z / headPoints.Count;

            var p = new CameraSpacePoint
            {
                X = x,
                Y = y,
                Z = z
            };

            return p;
        }

        public static void InitializeBodyHistory()
        {
            for (int i = 0; i < 10; i++)
            {
                List<Body> emptyBodies = new List<Body>();
                GlobVar.BodiesHistory.Enqueue(emptyBodies);
            }
        }

        public static void UpdateBodyHistory(List<Body> newBodies)
        {
            if (newBodies.Count > 0)
            {
                GlobVar.BodiesHistory.Enqueue(newBodies);
            }
            if (GlobVar.BodiesHistory.Count > 10)
            {
                GlobVar.BodiesHistory.Dequeue();
            }
        }

        public static Dictionary<Head,int> IdentifyHeads(List<Head> validatedCandidateHeads)
        {
            Dictionary<Head,int> identifiedHeads = new Dictionary<Head, int>();

            List<int> recentBodyIds = new List<int>();
            for (int i = GlobVar.BodiesHistory.Count - 1; i >= NumberOfTrackedFramesStartIndex; i--)
            {
                foreach (var body in GlobVar.BodiesHistory.ElementAt(i))
                {
                    if (!recentBodyIds.Contains(body.Id))
                    {
                        recentBodyIds.Add(body.Id);
                    }
                }
            }

            var avgLocationsLastHeads = new Dictionary<int, CameraSpacePoint>();

            foreach (var bodyId in recentBodyIds)
            {
                avgLocationsLastHeads.Add(bodyId,GetAverageHeadLocationLastTenFrames(bodyId));
                if (Diagnostics.ShowHeadAvgCenterLastTenFrames)
                {
                    if (!float.IsNaN(GetAverageHeadLocationLastTenFrames(bodyId).Z))
                    {
                        GraphicsUtils.DrawPoint(GetAverageHeadLocationLastTenFrames(bodyId));
                    }
                }
            }

            var avgDistancesFromCandidateHeadsToLastHeads = new Dictionary<Tuple<Head, int>, float>();
            
            foreach (var candidateHead in validatedCandidateHeads)
            {
                foreach (var avgLocationLastHead in avgLocationsLastHeads)
                {
                    var assignment = new Tuple<Head, int>(candidateHead,avgLocationLastHead.Key);
                    var distanceToCandidate = GlobUtils.GetEuclideanDistance(candidateHead.CenterPoint,
                        avgLocationLastHead.Value);

                    if (distanceToCandidate < Thresholds.LastFramesHeadMaxDistance)
                    {
                        avgDistancesFromCandidateHeadsToLastHeads.Add(assignment,distanceToCandidate);
                    }
                }
            }

            var sortedDistances = avgDistancesFromCandidateHeadsToLastHeads.OrderBy(i => i.Value);

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

        //public static int GetBodyIdFromHead(Head head) {
        //    float minDistanceHeads = float.MaxValue;
        //    int bodyId = -1;

        //    foreach (var bodies in GlobVar.BodiesHistory)
        //    {
        //        foreach (var body in bodies)
        //        {
        //            int currentBodyId = body.Id;

        //            CameraSpacePoint avgHeadPoint = GetAverageHeadLocationLastTenFrames(currentBodyId);

        //            if (Diagnostics.ShowHeadAvgCenterPoint)
        //            {
        //                GraphicsUtils.DrawPoint(avgHeadPoint);
        //            }

        //            float currentDistanceHeads = GlobUtils.GetEuclideanDistance(avgHeadPoint, head.CenterPoint);

        //            Console.WriteLine(currentDistanceHeads);
        //            if (currentDistanceHeads < minDistanceHeads && currentDistanceHeads < Thresholds.LastFramesHeadMaxDistance)
        //            {
        //                minDistanceHeads = currentDistanceHeads;
        //                bodyId = currentBodyId;
        //            }
        //        }
        //    }

        //    return bodyId;
        //}

        public static CameraSpacePoint GetAverageHeadLocationLastTenFrames(int newBodyId)
        {
            float sumX = 0;
            float sumY = 0;
            float sumZ = 0;

            float bodiesCount = 0;

            for (int i = 0; i < GlobVar.BodiesHistory.Count; i++)
            {
                var bodies = GlobVar.BodiesHistory.ElementAt(i);
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

        public static CameraSpacePoint GetAverageHeadLocationLastFrames(int newBodyId)
        {
            float sumX = 0;
            float sumY = 0;
            float sumZ = 0;

            float bodiesCount = 0;

            for (int i = NumberOfTrackedFramesStartIndex; i < GlobVar.BodiesHistory.Count; i++)
            {
                var bodies = GlobVar.BodiesHistory.ElementAt(i);
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

        public static CameraSpacePoint GetAverageTorsoLocationLastFrames(int newBodyId)
        {
            float sumX = 0;
            float sumY = 0;
            float sumZ = 0;

            int bodiesCount = 0;

            for (int i = NumberOfTrackedFramesStartIndex; i < GlobVar.BodiesHistory.Count; i++)
            {
                var bodies = GlobVar.BodiesHistory.ElementAt(i);
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

            //Console.WriteLine(bodiesCount);
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


            for (int i = NumberOfTrackedFramesStartIndex; i < GlobVar.BodiesHistory.Count; i++)
            {
                var bodies = GlobVar.BodiesHistory.ElementAt(i);
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
            for (int i = NumberOfTrackedFramesStartIndex; i < GlobVar.BodiesHistory.Count; i++)
            {
                foreach (var body in GlobVar.BodiesHistory.ElementAt(i))
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
            for (int i = NumberOfTrackedFramesStartIndex; i < GlobVar.BodiesHistory.Count; i++)
            {
                foreach (var body in GlobVar.BodiesHistory.ElementAt(i))
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
            for (int i = NumberOfTrackedFramesStartIndex; i < GlobVar.BodiesHistory.Count; i++)
            {
                foreach (var body in GlobVar.BodiesHistory.ElementAt(i))
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
            return GlobVar.BodiesHistory.Count > 0;
        }

        public static List<Body> GetBodiesFromHistory(int bodyId)
        {
            List<Body> bodiesWithId = new List<Body>();

            for (int i = 7; i < GlobVar.BodiesHistory.Count-1; i++)
            {
                foreach (var body in GlobVar.BodiesHistory.ElementAt(i))
                {
                    if (body.Id == bodyId)
                    {
                        bodiesWithId.Add(body);
                    }
                }
            }

            if (bodiesWithId.Count > 0)
            {
                
            }

            return bodiesWithId;

        }

        public static double GetTimeSpanLastTenFrames()
        {
            if (GlobVar.BodiesHistory.Count < 2)
            {
                return -1;
            }

            TimeSpan timeStampStart = GlobVar.BodiesHistory.ElementAt(0)[0].TimeStamp;
            TimeSpan timeStampStop = GlobVar.BodiesHistory.ElementAt(GlobVar.BodiesHistory.Count - 1)[0].TimeStamp;

            double deltaTime = timeStampStop.Subtract(timeStampStart).TotalSeconds;
            return deltaTime;
        }

    }


}
