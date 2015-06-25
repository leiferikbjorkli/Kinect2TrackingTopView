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
        public static int CalculateCenterPointHeadPoints(List<int> headPixels )
        {
            List<CameraSpacePoint> headPoints = new List<CameraSpacePoint>();

            for (int i = 0; i < headPixels.Count; i++)
            {
                CameraSpacePoint rawPoint = GlobVar.PointCloud[headPixels[i]];

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


            int xPixelCoordinateOfPoint = (int)Math.Round(((p.X * 1000) / GlobVar.MaxHorizontalWidth) * (GlobVar.ScaledFrameWidth / 2) + (GlobVar.ScaledFrameWidth / 2));
            int yPixelCoordinateOfPoint = (int)Math.Round(((-p.Y * 1000) / GlobVar.MaxVerticalHeight) * (GlobVar.ScaledFrameHeight / 2) + (GlobVar.ScaledFrameHeight / 2));

            return GlobUtils.GetIndex(xPixelCoordinateOfPoint, yPixelCoordinateOfPoint);
        }

        public static CameraSpacePoint CalculateCenterPointHeadPointsPerspective(List<int> headPixels)
        {
            List<CameraSpacePoint> headPoints = new List<CameraSpacePoint>();

            for (int i = 0; i < headPixels.Count; i++)
            {
                CameraSpacePoint rawPoint = GlobVar.PointCloud[headPixels[i]];

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


            //int xPixelCoordinateOfPoint = (int)Math.Round(((p.X * 1000) /
            //GlobVar.MaxHorizontalWidth) * (GlobVar.ScaledFrameWidth / 2) +
            //(GlobVar.ScaledFrameWidth / 2)); int yPixelCoordinateOfPoint =
            //(int)Math.Round(((-p.Y * 1000) / GlobVar.MaxVerticalHeight) *
            //(GlobVar.ScaledFrameHeight / 2) + (GlobVar.ScaledFrameHeight /
            //2));

            return p;
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

        public static int GetBodyIdFromHead(Head head)
        {
            float minDistanceHeads = float.MaxValue;
            int bodyId = -1;

            foreach (var bodies in GlobVar.BodiesHistory)
            {
                foreach (var body in bodies)
                {
                    int currentBodyId = body.Id;

                    CameraSpacePoint avgHeadPoint = GetAverageHeadLocationLastTenFrames(currentBodyId);

                    //Graphics.DrawPoint(avgHeadPoint);

                    float currentDistanceHeads = GlobUtils.GetEuclideanDistance(avgHeadPoint, head.CenterPoint);

                    if (currentDistanceHeads < minDistanceHeads && currentDistanceHeads < Thresholds.LastTenFramesHeadMaxDistance)
                    {
                        minDistanceHeads = currentDistanceHeads;
                        bodyId = currentBodyId;
                    }
                }
            }

            return bodyId;
        }

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
                X =sumX/bodiesCount,
                Y = sumY/bodiesCount,
                Z = sumZ/bodiesCount
            };
        }

        public static CameraSpacePoint GetAverageTorsoLocationLastTenFrames(int newBodyId)
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

            return new CameraSpacePoint
            {
                X = sumX / bodiesCount,
                Y = sumY / bodiesCount,
                Z = sumZ / bodiesCount
            };
        }

        public static CameraSpacePoint[] GetAverageHandLocationsLastTenFrames(int bodyId)
        {
            
            float sumXfirstHand = 0;
            float sumYfirstHand = 0;
            float sumZfirstHand = 0;

            float sumXsecondHand = 0;
            float sumYsecondHand = 0;
            float sumZsecondHand = 0;

            float firstHandCount = 0;
            float secondHandCount = 0;


            for (int i = 0; i < GlobVar.BodiesHistory.Count; i++)
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
                                } else if (hand.Id == 1)
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

            if (firstHand.Z == 0 || secondHand.Z == 0)
            {
                

            }

            return new CameraSpacePoint[]{firstHand,secondHand};
        }

        public static bool HasFirstHandTracking(int bodyId)
        {
            foreach (var bodies in GlobVar.BodiesHistory)
            {
                foreach (var body in bodies)
                {
                    if (body.Id == bodyId && body.Hands[0] != null)
                    {
                        return true;
                    }
                }
            }
            return false;
        }

        public static bool HasSecondHandTracking(int bodyId)
        {
            foreach (var bodies in GlobVar.BodiesHistory)
            {
                foreach (var body in bodies)
                {
                    if (body.Id == bodyId && body.Hands[1] != null)
                    {
                        return true;
                    }
                }
            }
            return false;
        }

        public static bool HasTorsoTracking(int bodyId)
        {
            foreach (var bodies in GlobVar.BodiesHistory)
            {
                foreach (var body in bodies)
                {
                    if (body.Id == bodyId && body.Torso != null)
                    {
                        return true;
                    }
                }
            }
            return false;
        }

        public static bool HasBodyTracking()
        {
            return GlobVar.BodiesHistory.Count > 0;
        }

        public static List<Body> GetBodiesFromHistory(int bodyId)
        {
            List<Body> bodiesWithId = new List<Body>();

            foreach (var bodies in GlobVar.BodiesHistory)
            {
                foreach (var body in bodies)
                {
                    if (body.Id == bodyId)
                    {
                        bodiesWithId.Add(body);
                    }
                }
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
