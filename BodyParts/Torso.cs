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
    class Torso
    {
        public CameraSpacePoint CenterPoint { get; set; }
        public CameraSpacePoint AvgCenterPoint { get; set; }

        public Torso(CameraSpacePoint centerPoint, CameraSpacePoint avgCenterPoint)
        {
            CenterPoint = centerPoint;
            AvgCenterPoint = avgCenterPoint;
        }
    }
}
