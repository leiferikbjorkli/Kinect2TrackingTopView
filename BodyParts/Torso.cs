//
// Copyright (c) Leif Erik Bjoerkli, Norwegian University of Science and Technology, 2015.
// Distributed under the MIT License.
// (See accompanying file LICENSE or copy at http://opensource.org/licenses/MIT)
//  

using Microsoft.Kinect;

namespace Kinect2TrackingTopView
{
    public class Torso
    {
        public CameraSpacePoint CenterPoint { get; private set; }
        public CameraSpacePoint AvgCenterPoint { get; private set; }

        public Torso(CameraSpacePoint centerPoint, CameraSpacePoint avgCenterPoint)
        {
            CenterPoint = centerPoint;
            AvgCenterPoint = avgCenterPoint;
        }
    }
}
