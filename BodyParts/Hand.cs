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
    class Hand
    {
        public List<CameraSpacePoint> Points { get; set; }
        public int Id { get; set; }
        public CameraSpacePoint CenterPoint { get; set; }

        public Hand(CameraSpacePoint centerPoint, int id)
        {
            CenterPoint = centerPoint;
            Id = id;
        }
    }
}
