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
        public const int maxDepth = 2150;
        public const int minDepth = 1000;
        public const float MaxDepthMeter = 2.15f;
        public const float MinDepthMeter = 1.0f;
        public const int frameWidth = 512;
        public const int frameHeight = 424;
        public const int frameLength = frameWidth * frameHeight;
        public const int scaledFrameWidth = frameWidth / 2;
        public const int scaledFrameHeight = frameHeight / 2;
        public const int scaledFrameLength = scaledFrameHeight * scaledFrameWidth;
        public const float horizontalFieldOfView = 70.6f;
        public const float verticalFieldOfView = 60.0f;
        public static double MaxHorizontalWidth = Math.Tan((GlobUtils.ToRadians(horizontalFieldOfView) / 2)) * maxDepth;
        public static double MaxVerticalHeight = Math.Tan((GlobUtils.ToRadians(verticalFieldOfView) / 2)) * maxDepth;



        public static CameraSpacePoint[] InvertedCloud = new CameraSpacePoint[scaledFrameLength];

        static public CameraSpacePoint[] scaledCameraSpacePoints = new CameraSpacePoint[scaledFrameLength];
        static public byte[] canvas = new byte[scaledFrameLength];
        static public float[] depthMap = new float[scaledFrameLength];
        static public Dictionary<int, Dictionary<int, float>> AdjacancyList = new Dictionary<int, Dictionary<int, float>>();
        static public List<Head> Heads = new List<Head>();


        static public KinectSensor KinectSensor = KinectSensor.GetDefault();
        static public CoordinateMapper CoordinateMapper = KinectSensor.CoordinateMapper;

    }
}
