//
// Written by Leif Erik Bjoerkli
//


using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace InteractionDetection
{
    class Diagnostics
    {
        // IDEA: Adapt size of headPixels to approx sensing height

        //only calculate energyid stable tracking
        
        public const bool LogTotalTime = false;
        public const bool LogTime = false;
        public const bool LogTimeGeodesic = false;
        public const bool LogTimeHaar = false;
        public const bool LogTimeMedian = false;    
        public const bool PrintFPS = false;
        
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

        private const int NumberOfHands = 2;
        private const int NumberOfBodiesInFrame = 1;


        private List<int> handsTrackedHistory;
        private List<TimeSpan> timestamps;
        private List<int> bodyTrackedHistory;

        public Diagnostics()
        {
            // number of tracked joints with the index being the frame
            handsTrackedHistory = new List<int>();

            // timestamp of frames
            timestamps = new List<TimeSpan>();

            // number of bodies that was tracked in the frames
            bodyTrackedHistory = new List<int>();

        }

        public List<TimeSpan> Timestamps
        {
            get { return timestamps; }
        }

        public List<int> HandsTrackedHistory
        {
            get { return handsTrackedHistory; }
        }

        public List<int> BodyTrackedHistory
        {
            get { return bodyTrackedHistory; }
        }

        public void AddNewBodyFrameDiagnostics(List<Body> bodies, TimeSpan timestamp)
        {
            int bodiesTracked = 0;
            int handsTracked = 0;
            foreach (var body in bodies)
            {
                bodiesTracked++;
                foreach (var hand in body.Hands)
                {
                    if (hand != null)
                    {
                        handsTracked++;
                    }
                }
            }

            Timestamps.Add(timestamp);
            BodyTrackedHistory.Add(bodiesTracked);
            HandsTrackedHistory.Add(handsTracked);
        }

        public void Draw(Body body)
        {
            if (Diagnostics.ShowHandCenterPoint)
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

            if (Diagnostics.ShowHandAvgPoint)
            {
                Hand firstHand = body.Hands[0];
                if (firstHand != null && !float.IsNaN(firstHand.AvgCenterPointLastFiveFrames.Z))
                {
                    GraphicsUtils.DrawPoint(firstHand.AvgCenterPointLastFiveFrames);
                }
                Hand secondHand = body.Hands[1];
                if (secondHand != null && !float.IsNaN(secondHand.AvgCenterPointLastFiveFrames.Z))
                {
                    GraphicsUtils.DrawPoint(secondHand.AvgCenterPointLastFiveFrames);
                }
            }

            if (Diagnostics.ShowTorsoCenterPoint)
            {
                if (body.Torso != null && !float.IsNaN(body.Torso.CenterPoint.Z))
                {
                    GraphicsUtils.DrawPoint(body.Torso.CenterPoint);
                }
            }
            if (Diagnostics.ShowTorsoAvgCenterPoint)
            {
                if (body.Torso != null && !float.IsNaN(body.Torso.AvgCenterPoint.Z))
                {
                    GraphicsUtils.DrawPoint(body.Torso.AvgCenterPoint);
                } 
            }

            

            if (Diagnostics.ShowHeadCenterPoint)
            {
            
                 GraphicsUtils.DrawPoint(body.Head.CenterPoint);
            }
        }

        /// <summary>
        /// Calculates the average amount of tracked and inferred joints, as well as the amount of frames a body was tracked.
        /// </summary>
        public void CalculateTrackingSuccess()
        {
            int totalHandsTracked = 0;
            int totalBodiesTracked = 0;
            int numberOfFrames = BodyTrackedHistory.Count;

            for (int i = 0; i < numberOfFrames; i++)
            {
                totalBodiesTracked += BodyTrackedHistory[i];
                totalHandsTracked += HandsTrackedHistory[i];
            }
                
            int totalNumberPossibleHands = NumberOfHands * totalBodiesTracked;

            float bodyTrackedAmount = (float) totalBodiesTracked / (float) numberOfFrames*NumberOfBodiesInFrame;
            float handsTrackedAmount = (float)totalHandsTracked / (float)totalNumberPossibleHands;

            Console.WriteLine("BodyTrackedSuccess: {0}%", bodyTrackedAmount * 100);
            Console.WriteLine("HandsTrackedSuccess: {0}%", handsTrackedAmount*100);
        }

    }
}
