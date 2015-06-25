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
        public List<CameraSpacePoint> Points { get; set; }
        public CameraSpacePoint CenterPoint { get; set; }

        public Torso(CameraSpacePoint centerPoint)
        {
            CenterPoint = centerPoint;
        }
    }
}
