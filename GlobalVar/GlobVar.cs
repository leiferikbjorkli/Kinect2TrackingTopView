//
// Written by Leif Erik Bjoerkli
//


using System;
using System.Collections.Generic;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using Microsoft.Kinect;

namespace InteractionDetection
{
    static class GlobVar
    {
        public const float MaxXWidth = 1f;
        public const float MaxYWidth = 1f;
        public const int MaxDepth = 3580;
        public const int MinDepth = 2500;
        //public const int MaxDepth = 3700;
        //public const int MaxDepth = 2115;
        //public const int MinDepth = 1000;
        public const float MaxDepthMeter = ((float)MaxDepth)/1000;
        public const float MinDepthMeter = ((float)MinDepth)/1000;
        public const int FrameWidth = 512;
        public const int FrameHeight = 424;
        public const int FrameLength = FrameWidth * FrameHeight;
        public const int ScaledFrameWidth = FrameWidth / 2;
        public const int ScaledFrameHeight = FrameHeight / 2;
        public const int ScaledFrameLength = ScaledFrameHeight * ScaledFrameWidth;
        public const float HorizontalFieldOfView = 70.6f;
        public const float VerticalFieldOfView = 60.0f;
        public static double MaxHorizontalWidth = Math.Tan((GlobUtils.ToRadians(HorizontalFieldOfView) / 2)) * MaxDepth;
        public static double MaxVerticalHeight = Math.Tan((GlobUtils.ToRadians(VerticalFieldOfView) / 2)) * MaxDepth;

        static public CameraSpacePoint[] ScaledCameraSpacePoints = new CameraSpacePoint[ScaledFrameLength];
        static public byte[] Canvas = new byte[ScaledFrameLength];

        static public Dictionary<int, Dictionary<int, float>> AdjacancyList = new Dictionary<int, Dictionary<int, float>>();

        static public Queue<List<Body>> BodiesHistory = new Queue<List<Body>>();
        public static int MaxBodyCount;

        public static CameraSpacePoint[] MedianFilteredPointCloud = new CameraSpacePoint[ScaledFrameLength];

        public static WriteableBitmap IntensityBitmap = new WriteableBitmap(GlobVar.ScaledFrameWidth, GlobVar.ScaledFrameHeight, 96.0, 96.0, PixelFormats.Gray8, null);

        public static CameraSpacePoint[] PointCloud = new CameraSpacePoint[GlobVar.ScaledFrameLength];

        public static CoordinateMapper CoordinateMapper;

        public static Dictionary<int, double> BodyEnergies = new Dictionary<int, double>();

        public static List<Head> ValidatedCandidateHeads;
    }
}
