using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;

namespace InteractionDetection
{
    static class GlobVar
    {
        public const int maxDepth = 2300;
        public const int minDepth = 1000;
        public const int frameWidth = 512;
        public const int frameHeight = 424;
        public const int frameLength = frameWidth * frameHeight;
        public const int scaledFrameWidth = frameWidth / 2;
        public const int scaledFrameHeight = frameHeight / 2;
        public const int scaledFrameLength = scaledFrameHeight * scaledFrameWidth;
        public const float horizontalFieldOfView = 70.6f;
        public const float verticalFieldOfView = 60.0f;
        static public CameraSpacePoint[] scaledCameraSpacePoints = new CameraSpacePoint[scaledFrameLength];
        static public byte[] canvas = new byte[scaledFrameLength];
    }
}
