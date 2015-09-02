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
    /// <summary>
    /// Class that logs and visualizes runtime information to the console and the monitoring window.
    /// </summary>
    static class Logger
    {
        public const bool LogTimeTotal = false;
        public const bool LogTimeScaling = false;
        public const bool LogTimeMedian = false;
        public const bool LogTimeHaar = false;
        public const bool LogTimeClassificationValidation = false;
        public const bool LogTimeGeodesic = false;
        public const bool LogTimeCreateBodies = false;
        public const bool PrintFps = false;
        public const bool PrintTotalFps = true;

        public const bool ShowHaarInnerRects = false;
        public const bool ShowHaarOuterRects = false;
        public const bool ShowCandidateHeadpixel = false;
        public const bool ShowTopHeadpixel = false;
        public const bool ShowHeadpixelsBeforeValidation = false;
        public const bool ShowTopHeadPixelAfterGrouping = false;

        public const bool ShowValidatedTopHeadpixel = false;
        public const bool ShowValidatedHeadPixels = false;
        public const bool ShowGeodesicGraph = false;

        public const bool ShowHeadRegionPixels = false;
        public const bool ShowHeadCenterPoint = false;
        public const bool ShowHeadAvgCenterPoint = true;
        public const bool ShowHeadAvgCenterLastTenFrames = false;

        public const bool ShowTorsoPixels = false;
        public const bool ShowTorsoCenterPoint = true;
        public const bool ShowTorsoAvgCenterPoint = false;

        public const bool ShowHandPixels = false;
        public const bool ShowHandCenterPoint = false;
        public const bool ShowHandAvgPoint = false;

        public const bool DrawEnergyValue = false;

        /// <summary>
        /// Calculate frames per second.
        /// </summary>
        public static double CalculateFps(TimeSpan currentTime, TimeSpan lastTime)
        {
            double deltaTime = currentTime.Subtract(lastTime).TotalMilliseconds;
            return 1000 / deltaTime;
        }

        /// <summary>
        /// Calculate average frames per second.
        /// </summary>
        public static double CalculateAverageFps(List<TimeSpan> timestamps)
        {
            var totalTime = timestamps[timestamps.Count - 1].Subtract(timestamps[0]);
            return timestamps.Count/totalTime.TotalSeconds;
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
