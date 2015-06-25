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
        public const bool LogTime = false;
        public const bool LogTimeHaar = false;
        public const bool ShowHaarRects = false;
        public const bool PrintFPS = false;
        public const bool ShowGeodesicGraph = false;
        public const bool ShowHandPixels = false;
        public const bool ShowHeadPixels = true;

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

            timestamps.Add(timestamp);
            bodyTrackedHistory.Add(bodiesTracked);
            handsTrackedHistory.Add(handsTracked);
        }

        /// <summary>
        /// Calculates the average amount of tracked and inferred joints, as well as the amount of frames a body was tracked.
        /// </summary>
        public void CalculateTrackingSuccess()
        {
            int totalHandsTracked = 0;
            int totalBodiesTracked = 0;
            int numberOfFrames = bodyTrackedHistory.Count;

            for (int i = 0; i < numberOfFrames; i++)
            {
                totalBodiesTracked += bodyTrackedHistory[i];
                totalHandsTracked += handsTrackedHistory[i];
            }
                
            int totalNumberPossibleHands = NumberOfHands * totalBodiesTracked;

            float bodyTrackedAmount = (float) totalBodiesTracked / (float) numberOfFrames*NumberOfBodiesInFrame;
            float handsTrackedAmount = (float)totalHandsTracked / (float)totalNumberPossibleHands;

            Console.WriteLine("BodyTrackedSuccess: {0}%", bodyTrackedAmount * 100);
            Console.WriteLine("HandsTrackedSuccess: {0}%", handsTrackedAmount*100);
        }

    }
}
