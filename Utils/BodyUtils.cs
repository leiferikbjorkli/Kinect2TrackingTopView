using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;

namespace InteractionDetection
{
    static class BodyUtils
    {
        public static void CalculateGazeDirection(Head head)
        {

            List<int> headPixels = head.HeadPixels;
            CameraSpacePoint[] headPoints = new CameraSpacePoint[headPixels.Count];

            for (int i = 0; i < headPoints.Length; i++)
            {
                headPoints[i] = GlobVar.scaledCameraSpacePoints[headPixels[i]];
            }

            float x = 0;
            float y = 0;
            float z = 0;

            for (int i = 0; i < headPoints.Length; i++)
            {
                x += headPoints[i].X;
                y += headPoints[i].Y;
                z += headPoints[i].Z;
            }

            x = x / headPoints.Length;
            y = y / headPoints.Length;
            z = z / headPoints.Length;

            Graphics.DrawPointNoPointcloud(x,y,z);

            //MathUtils.LinearRegression(headPoints);

        }

        public static void CalculateCenterPointHeadPoints(Head head)
        {
            List<int> headPixels = head.HeadPixels;
            CameraSpacePoint[] headPoints = new CameraSpacePoint[headPixels.Count];

            for (int i = 0; i < headPoints.Length; i++)
            {
                headPoints[i] = GlobVar.scaledCameraSpacePoints[headPixels[i]];
            }

            float x = 0;
            float y = 0;
            float z = 0;

            for (int i = 0; i < headPoints.Length; i++)
            {
                x += headPoints[i].X;
                y += headPoints[i].Y;
                z += headPoints[i].Z;
            }

            x = x / headPoints.Length;
            y = y / headPoints.Length;
            z = z / headPoints.Length;


            var p = new CameraSpacePoint
            {
                X = x,
                Y = y,
                Z = z
            };
            var depthSpacePoint = GlobVar.CoordinateMapper.MapCameraPointToDepthSpace(p);
            head.CenterPoint = GlobUtils.GetIndex((int)depthSpacePoint.X/2, (int)depthSpacePoint.Y/2);
        }
    }
}
