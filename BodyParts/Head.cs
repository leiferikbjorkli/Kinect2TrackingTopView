//
// Copyright (c) Leif Erik Bjoerkli, Norwegian University of Science and Technology, 2015.
// Distributed under the MIT License.
// (See accompanying file LICENSE or copy at http://opensource.org/licenses/MIT)
//  

using System;
using System.Collections.Generic;
using Microsoft.Kinect;

namespace Kinect2TrackingTopView
{
    public class Head
    {
        public List<int> HeadPointIndexes { get; private set; }
        public int CenterPointIndex;
        public CameraSpacePoint CenterPoint { get; private set; }
        public CameraSpacePoint AvgCenterPoint{ get; set; }
        public readonly int HighestPointIndex;

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

        public int AddHeadPixels(List<int> headPixels)
        {
            HeadPointIndexes = headPixels;
            CenterPoint = BodyUtils.CalculateAveragePointFromValidPoints(headPixels);

            var depthSpacePoint = GlobVar.CoordinateMapper.MapCameraPointToDepthSpace(CenterPoint);

            CenterPointIndex = GlobUtils.GetIndex((int)Math.Round(depthSpacePoint.X) / 2, (int)Math.Round(depthSpacePoint.Y) / 2);

            return CenterPointIndex;
        }
    }
}
