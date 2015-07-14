//
// Written by Leif Erik Bjoerkli
//


using System;
using System.Collections.Generic;
using System.Security.Cryptography.X509Certificates;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using Microsoft.Kinect;

namespace InteractionDetection
{
    static class GlobVar
    {

        //add { get; set; }
        
        public static double HalfMaxHorizontalWidth = Math.Tan((GlobUtils.ToRadians(HorizontalFieldOfView) / 2)) * MaxDepth;
        public static double HalfMaxVerticalHeight = Math.Tan((GlobUtils.ToRadians(VerticalFieldOfView) / 2)) * MaxDepth;
        public const float RadiusHumanHeadMm = 87.5f;
        public const float RadiusHumanHead = 0.0875f;
        public const float ApproxSensingDepth = MaxDepth - 1800;
        public static double PixelsPerMmAtSensingHeight = ScaledFrameWidth / (Math.Tan((GlobUtils.ToRadians(HorizontalFieldOfView) / 2)) * ApproxSensingDepth * 2);
        public static int HaarInnerWindowSize = (int)(PixelsPerMmAtSensingHeight * RadiusHumanHeadMm * 2);
        //public static int HaarInnerWindowSize = 8;
        public static int HaarOuterWindowSize = HaarInnerWindowSize*3;
        //public static int HaarSmallestWindowSize =
        //    (int)(PixelsPerMmAtSensingHeight * Math.Sqrt((RadiusHumanHeadMm * RadiusHumanHeadMm - (RadiusHumanHeadMm * (2.0 / 3.0)) * (RadiusHumanHeadMm * (2.0 / 3.0))) * 4));

        public const float MaxXWidth = 3f;
        public const float MaxYWidth = 3f;
        public const int MaxDepth = 4100;
        public const int MinDepth = 0;
        //public const int MaxDepth = 3630; public const int MinDepth = 2200;
        //public const int MaxDepth = 3700; public const int MaxDepth = 2115;
        //public const int MinDepth = 1000;
        public const float MaxDepthMeter = ((float)MaxDepth)/1000;
        public const float MinDepthMeter = ((float)MinDepth)/1000;
        public const int FrameWidth = 512;
        public const int FrameHeight = 424;
        public const int ScaledFrameWidth = FrameWidth / 2;
        public const int ScaledFrameHeight = FrameHeight / 2;
        public const int ScaledFrameLength = ScaledFrameHeight * ScaledFrameWidth;
        public const float HorizontalFieldOfView = 70.6f;
        public const float VerticalFieldOfView = 60.0f;
        

        static public CameraSpacePoint[] ScaledCameraSpacePoints = new CameraSpacePoint[ScaledFrameLength];
        static public byte[] Canvas = new byte[ScaledFrameLength];
        static public int[] HeatCanvas = new int[ScaledFrameLength];

        static public Dictionary<int, Dictionary<int, float>> AdjacancyList = new Dictionary<int, Dictionary<int, float>>();

        static public Queue<List<Body>> BodiesHistory = new Queue<List<Body>>();
        public static int MaxBodyCount;

        public static CameraSpacePoint[] SubtractedFilteredPointCloud = new CameraSpacePoint[ScaledFrameLength];
        
        public static CameraSpacePoint[] PreviousFrame = new CameraSpacePoint[ScaledFrameLength];

        public static WriteableBitmap IntensityBitmap = new WriteableBitmap(GlobVar.ScaledFrameWidth, GlobVar.ScaledFrameHeight, 96.0, 96.0, PixelFormats.Gray8, null);

        public static CoordinateMapper CoordinateMapper;

        public static Dictionary<int, double> BodyEnergies = new Dictionary<int, double>();

        public static List<Head> ValidatedCandidateHeads;

        public static int InitialFrameCounter;
    }
}
