//
// Copyright (c) Leif Erik Bjoerkli, Norwegian University of Science and Technology, 2015.
// Distributed under the MIT License.
// (See accompanying file LICENSE or copy at http://opensource.org/licenses/MIT)
//  

using System;
using System.Collections.Generic;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using Microsoft.Kinect;

namespace Kinect2TrackingTopView
{
    public static class GlobVar
    {
        private const float HorizontalFieldOfView = 70.6f;
        private const float VerticalFieldOfView = 60f;
        public const int FrameWidth = 512;
        public const int FrameHeight = 424;

        public const int ScaledFrameWidth = FrameWidth/2;
        public const int ScaledFrameHeight = FrameHeight/2;
        public const int ScaledFrameLength = ScaledFrameHeight*ScaledFrameWidth;

        // Sets the number of frames that is included in the background temporal
        // median image
        public const int TemporalFrameCounter = 30;

        // Global variable that stores the max amount of detected bodies in a
        // session for labeling purposes
        public static int MaxBodyCount = 0;

        // Average radius of a human head
        private const float RadiusHumanHead = 0.0875f;

        public const float MaxSensingDepth = 4.1f;
        public const float MinSensingDepth = 0;

        // Derive the approximate optimal size of the haar-windows
        private const float ApproxSensingDepth = (MaxSensingDepth*1000) - 1700;
        private static readonly double PixelsPerMmAtSensingHeight = ScaledFrameWidth / (Math.Tan((GlobUtils.ToRadians(HorizontalFieldOfView) / 2)) * ApproxSensingDepth * 2);
        public static readonly int HaarInnerWindowWidth = (int)(PixelsPerMmAtSensingHeight * (RadiusHumanHead*1000) * 2);
        public static readonly int HaarOuterWindowWidth = HaarInnerWindowWidth*3;

        static public readonly byte[] GraphicsCanvas = new byte[ScaledFrameLength];
        static public int[] HeatCanvas = new int[ScaledFrameLength];

        public static CameraSpacePoint[] SubtractedFilteredPointCloud = new CameraSpacePoint[ScaledFrameLength];
        
        public static readonly WriteableBitmap IntensityBitmap = new WriteableBitmap(ScaledFrameWidth, ScaledFrameHeight, 96.0, 96.0, PixelFormats.Gray8, null);

        public static CoordinateMapper CoordinateMapper;

        public static List<Head> ValidatedCandidateHeads;

        public static List<TimeSpan> TimeStamps;

    }
}
