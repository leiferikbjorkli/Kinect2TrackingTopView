//
// Copyright (c) Leif Erik Bjoerkli, Norwegian University of Science and Technology, 2015.
// Distributed under the MIT License.
// (See accompanying file LICENSE or copy at http://opensource.org/licenses/MIT)
//  

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Kinect2TrackingTopView
{
    static class Debugger
    {
        public const bool LogTotalTime = false;
        public const bool LogTime = false;
        public const bool LogTimeGeodesic = false;
        public const bool LogTimeHaar = false;
        public const bool LogTimeMedian = false;
        public const bool PrintFps = false;

        public const bool ShowHaarRects = true;
        public const bool ShowCandidateHeadpixel = false;
        public const bool ShowTopHeadpixel = false;
        public const bool ShowHeadpixelsBeforeValidation = false;
        public const bool ShowTopHeadPixelAfterGrouping = false;

        public const bool ShowValidatedTopHeadpixel = false;
        public const bool ShowValidatedHeadPixels = false;
        public const bool ShowGeodesicGraph = false;
        public const bool ShowTorso = false;

        public const bool ShowHeadCenterPoint = false;
        public const bool ShowHeadAvgCenterPoint = true;
        public const bool ShowHeadAvgCenterLastTenFrames = false;

        public const bool ShowTorsoCenterPoint = false;
        public const bool ShowTorsoAvgCenterPoint = true;

        public const bool ShowHandPixels = false;
        public const bool ShowHandCenterPoint = false;
        public const bool ShowHandAvgPoint = false;

        /// <summary>
        /// Calculate frames per second.
        /// </summary>
        public static double CalculateFps(TimeSpan currentTime, TimeSpan lastTime)
        {
            double deltaTime = currentTime.Subtract(lastTime).TotalMilliseconds;
            return 1000 / deltaTime;
        }

        /// <summary>
        /// Draw bodyelements in detection frame for debugging purposes.
        /// </summary>
        public static void Draw(Body body)
        {
            if (ShowHandCenterPoint)
            {
                Hand firstHand = body.Hands[0];
                if (firstHand != null)
                {
                    GraphicsUtils.DrawPoint(firstHand.CenterPoint);
                }
                Hand secondHand = body.Hands[1];
                if (secondHand != null)
                {
                    GraphicsUtils.DrawPoint(secondHand.CenterPoint);
                }
            }
            if (ShowHandAvgPoint)
            {
                Hand firstHand = body.Hands[0];
                if (firstHand != null && !float.IsNaN(firstHand.AvgCenterPointLastFrames.Z))
                {
                    GraphicsUtils.DrawPoint(firstHand.AvgCenterPointLastFrames);
                }
                Hand secondHand = body.Hands[1];
                if (secondHand != null && !float.IsNaN(secondHand.AvgCenterPointLastFrames.Z))
                {
                    GraphicsUtils.DrawPoint(secondHand.AvgCenterPointLastFrames);
                }
            }
            if (ShowTorsoCenterPoint)
            {
                if (body.Torso != null && !float.IsNaN(body.Torso.CenterPoint.Z))
                {
                    GraphicsUtils.DrawPoint(body.Torso.CenterPoint);
                }
            }
            if (ShowTorsoAvgCenterPoint)
            {
                if (body.Torso != null && !float.IsNaN(body.Torso.AvgCenterPoint.Z))
                {
                    GraphicsUtils.DrawPoint(body.Torso.AvgCenterPoint);
                }
            }
            if (ShowHeadCenterPoint)
            {
                GraphicsUtils.DrawPoint(body.Head.CenterPoint);
            }
        }
    }
}
