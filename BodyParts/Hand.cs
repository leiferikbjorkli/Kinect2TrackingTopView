//
// Copyright (c) Leif Erik Bjoerkli, Norwegian University of Science and Technology, 2015.
// Distributed under the MIT License.
// (See accompanying file LICENSE or copy at http://opensource.org/licenses/MIT)
//  

using Microsoft.Kinect;

namespace Kinect2TrackingTopView
{
    public class Hand
    {
        public int Id { get; private set; }
        public CameraSpacePoint CenterPoint { get; private set; }
        public CameraSpacePoint AvgCenterPointLastFrames { get; private set; }

        public Hand(CameraSpacePoint centerPoint, int id, CameraSpacePoint avgCenterPointLastFrames)
        {
            CenterPoint = centerPoint;
            AvgCenterPointLastFrames = avgCenterPointLastFrames;
            Id = id;
        }
    }
}
