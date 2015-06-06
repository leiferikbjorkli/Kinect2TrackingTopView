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

        public static Body GetLastFrameBody(Body body)
        {
            int indexHead = body.Head.CenterPoint;

            List<Body> bodies = GlobVar.Bodies;

            Body lastBody = null;
            float minDistanceHeads = float.MaxValue;

            for (int i = 0; i < bodies.Count; i++)
            {
                Body currentBody = bodies[i];
                float currentDistanceHeads = GlobUtils.GetEuclideanDistance(currentBody.Head.CenterPoint, indexHead);
                if (currentDistanceHeads < minDistanceHeads && currentDistanceHeads < Thresholds.MaxDistanceLastFrameHead)
                {
                    lastBody = currentBody;
                    minDistanceHeads = currentDistanceHeads;
                }
            }
            return lastBody;
        }

        public static void GroupShoulderPoints(List<int> shoulderPoints, int headCenterPoint, Point avgHandPoint, Body body)
        {

            Point headPoint = GlobUtils.GetPoint(headCenterPoint); 
            int headX = headPoint.x; 
            int headY = headPoint.y;

            Vector handVector = new Vector(avgHandPoint.x - headX, avgHandPoint.y - headY);

            var rightShoulderPoints = new List<int>();
            var leftShoulderPoints = new List<int>();


            for (int i = 0; i < shoulderPoints.Count; i++)
            {
                int index = shoulderPoints[i];
                Point p = GlobUtils.GetPoint(index);
                int shoulderX = p.x;
                int shoulderY = p.y;

                Vector pointVec = new Vector(shoulderX - headX, shoulderY - headY);

                if (Vector.AngleBetween(handVector,pointVec) > 0)
                {
                    rightShoulderPoints.Add(index);
                }
                else if (Vector.AngleBetween(handVector, pointVec) < 0)
                {
                    leftShoulderPoints.Add(index);
                }
            }


            if (leftShoulderPoints.Count == 0 || rightShoulderPoints.Count == 0)
            {
                Body lastBody = GetLastFrameBody(body);
                if (lastBody != null)
                {
                    body.AddLeftShoulder(lastBody.LeftShoulder);
                    body.AddRightShoulder(lastBody.RightShoulder); 
                }
            }
            else
            {
                LeftShoulder leftShoulder = new LeftShoulder();
                leftShoulder.Points = leftShoulderPoints;
                body.AddLeftShoulder(leftShoulder);

                RightShoulder rightShoulder = new RightShoulder();
                rightShoulder.Points = rightShoulderPoints;
                body.AddRightShoulder(rightShoulder);
                
            }
        }

        public static void GroupHandPoints(int headPoint,List<int> handPoints, Dictionary<int,int> minimumPath,Body body)
        {
            LeftHand leftHand = new LeftHand();
            RightHand rightHand = new RightHand();

            List<int> leftHandPoints = new List<int>();
            List<int> rightHandPoints = new List<int>();

            for (int i = 0; i < handPoints.Count; i++)
            {
                int handPoint = handPoints[i];
                int[] minPath = DijkstraUtils.GetMinimumPath(headPoint, handPoint, minimumPath);

                for (int j = 0; j < minPath.Length; j++)
                {
                    int pathPoint = minPath[j];
                    if (body.RightShoulder.Points.Contains(pathPoint))
                    {
                        rightHandPoints.Add(handPoint);
                        break;
                    }
                    if (body.LeftShoulder.Points.Contains(pathPoint))
                    {
                        leftHandPoints.Add(handPoint);
                        break;
                    }
                }
            }

            if (leftHandPoints.Count>0)
            {
                leftHand.Points = leftHandPoints;
                body.AddLeftHand(leftHand);
            }
            if (rightHandPoints.Count>0)
            {
                rightHand.Points = rightHandPoints;
                body.AddRightHand(rightHand);
            }
        }
        
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

        //public static void GroupHandPoints2(List<int> handPoints,int headCenterPoint, LeftHand leftHand, RightHand rightHand)
        //{
        //    Point headPoint = GlobUtils.GetPoint(headCenterPoint);
        //    int headX = headPoint.x;
        //    int headY = headPoint.y;

        //    var firstHandPoints = new List<int>();
        //    var secondHandPoints = new List<int>();

        //    firstHandPoints.Add(handPoints[0]);
        //    var firstHandPointRepresentative = GlobUtils.GetPoint(firstHandPoints[0]);
        //    var firstHandX = firstHandPointRepresentative.x;
        //    var firstHandY = firstHandPointRepresentative.y;

        //    Vector firstPointVec = new Vector(firstHandX - headX, firstHandY - headY);

        //    for (int i = 1; i < handPoints.Count; i++)
        //    {
        //        int pointIndex = handPoints[i];
        //        Point handPoint = GlobUtils.GetPoint(pointIndex);
        //        int handX = handPoint.x;
        //        int handY = handPoint.y;

        //        Vector pointVec = new Vector(handX-headX,handY-headY);

        //        var t = Math.Abs(Vector.Multiply(pointVec, firstPointVec));
                
        //        if (Math.Abs(Vector.AngleBetween(pointVec,firstPointVec)) < Thresholds.HandGroupingAngle)
        //        {
        //            firstHandPoints.Add(pointIndex);
        //        }
        //        else
        //        {
        //            secondHandPoints.Add(pointIndex);
        //        }
        //    }


        //    if (IsFirstHandRightHand(headCenterPoint,firstHandPoints[0],secondHandPoints[0]))
        //    {
        //        rightHand.Points = firstHandPoints;
        //        leftHand.Points = secondHandPoints;
        //    }
        //    else
        //    {
        //        rightHand.Points = secondHandPoints;
        //        leftHand.Points = firstHandPoints;
        //    }

        //}

        //public static bool IsFirstHandRightHand(int headCenterPoint, int firstHandRepresentative, int secondHandRepresentative)
        //{
        //    Point headPoint = GlobUtils.GetPoint(headCenterPoint);
        //    int headX = headPoint.x;
        //    int headY = headPoint.y;

        //    Point firstHandPoint = GlobUtils.GetPoint(firstHandRepresentative);
        //    int firstHandX = firstHandPoint.x;
        //    int firstHandY = firstHandPoint.y;

        //    Point secondHandPoint = GlobUtils.GetPoint(secondHandRepresentative);
        //    int secondHandX = secondHandPoint.x;
        //    int secondHandY = secondHandPoint.y;

        //    Graphics.DrawPoint(firstHandX,firstHandY);
        //    //Graphics.DrawPoint(secondHandX,secondHandY);

        //    Vector firstVector = new Vector(firstHandX - headX, firstHandY - headY);
        //    Vector secondVector = new Vector(secondHandX - headX, secondHandY - headY);

        //    if (Vector.Multiply(firstVector,secondVector) > 0)
        //    {
        //        return false;
        //    }
        //    return true;

        //}

        //public static void CalculateGazeDirection(Head head)
        //{

        //    List<int> headPixels = head.HeadPixels;
        //    CameraSpacePoint[] headPoints = new CameraSpacePoint[headPixels.Count];

        //    for (int i = 0; i < headPoints.Length; i++)
        //    {
        //        headPoints[i] = GlobVar.ScaledCameraSpacePoints[headPixels[i]];
        //    }

        //    float x = 0;
        //    float y = 0;
        //    float z = 0;

        //    for (int i = 0; i < headPoints.Length; i++)
        //    {
        //        x += headPoints[i].X;
        //        y += headPoints[i].Y;
        //        z += headPoints[i].Z;
        //    }

        //    x = x / headPoints.Length;
        //    y = y / headPoints.Length;
        //    z = z / headPoints.Length;

        //    Graphics.DrawPointNoPointcloud(x,y,z);

        //    //MathUtils.LinearRegression(headPoints);

        //}

    }
}
