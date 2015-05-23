using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;

namespace InteractionDetection
{
    class PixelVertex
    {
        private CameraSpacePoint cameraSpacePoint;
        private int componentID;

        public PixelVertex(CameraSpacePoint cameraSpacePoint, int id)
        {
            this.cameraSpacePoint = cameraSpacePoint;
            this.componentID = id;

        }

        public int ComponentID
        {
            get { return componentID; }
            set { componentID = value; }
        }

        public CameraSpacePoint CameraSpacePoint
        {
            get { return cameraSpacePoint; }
            set { cameraSpacePoint = value; }
        }

    }
}
