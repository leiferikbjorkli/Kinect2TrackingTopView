//
// Written by Leif Erik Bjoerkli
//


using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;

namespace InteractionDetection
{
    class Head
    {
        public List<int> HeadPointIndexes { get; private set; }
        public int Label { get; private set; }
        public int CenterPointIndex;
        public Point CartesianCenterPoint;
        public CameraSpacePoint CenterPoint { get; set; }
        public CameraSpacePoint HighestPoint { get; set; }
        private CameraSpacePoint _avgCenterPoint;
        public int HighestPointIndex;

        public CameraSpacePoint AvgCenterPoint
        {
            get { return _avgCenterPoint; }
            set { _avgCenterPoint = value; }
        }


        public Head(int highestPointIndex)
        {
            HighestPointIndex = highestPointIndex;
            AvgCenterPoint = new CameraSpacePoint()
            {
                X = float.NaN,
                Y = float.NaN,
                Z = float.NaN
            };
        }

        public int AddHeadPixels(List<int> headPixels )
        {
            HeadPointIndexes = headPixels;
            CenterPoint = BodyUtils.CalculateCenterPointFromValidPoints(headPixels);

            var depthSpacePoint = GlobVar.CoordinateMapper.MapCameraPointToDepthSpace(CenterPoint);

            CenterPointIndex = GlobUtils.GetIndex((int)Math.Round(depthSpacePoint.X) / 2, (int)Math.Round(depthSpacePoint.Y) / 2);

            if (CenterPointIndex == -1)
            {
                return -1;
            }

            if (float.IsInfinity(GlobVar.SubtractedFilteredPointCloud[CenterPointIndex].X) || float.IsInfinity(GlobVar.SubtractedFilteredPointCloud[CenterPointIndex].Y) || GlobVar.SubtractedFilteredPointCloud[CenterPointIndex].Z == GlobVar.MaxDepthMeter)
            {

            }

            return 1;
        }
    }
}
