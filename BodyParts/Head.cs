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

        public Head(CameraSpacePoint highestPoint)
        {
            HighestPoint = highestPoint;
        }

        public int AddHeadPixels(List<int> headPixels )
        {
            HeadPointIndexes = headPixels;
            CenterPoint = BodyUtils.CalculateCenterPointHeadPointsPerspective(headPixels);

            var depthSpacePoint = GlobVar.CoordinateMapper.MapCameraPointToDepthSpace(CenterPoint);

            CenterPointIndex = GlobUtils.GetIndex((int)Math.Round(depthSpacePoint.X) / 2, (int)Math.Round(depthSpacePoint.Y) / 2);

            
            if (CenterPointIndex == -1)
            {
                return -1;
            }

            CartesianCenterPoint = GlobUtils.GetPoint(CenterPointIndex);
            return 1;
        }

        

    }
}
