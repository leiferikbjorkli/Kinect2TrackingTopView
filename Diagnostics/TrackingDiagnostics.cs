//
// Copyright (c) Leif Erik Bjoerkli, Norwegian University of Science and Technology, 2015.
// Distributed under the MIT License.
// (See accompanying file LICENSE or copy at http://opensource.org/licenses/MIT)
//  

using System;
using System.Collections.Generic;

namespace Kinect2TrackingTopView
{
    /// <summary>
    /// Class that keeps track of and calculates tracking success in a sensing session.
    /// </summary>
    class TrackingDiagnostics
    {
        private const int NumberOfHands = 2;
        private const int NumberOfBodiesInFrame = 1;

        public TrackingDiagnostics()
        {
            HandsTrackedHistory = new List<int>();

            Timestamps = new List<TimeSpan>();

            HeadTrackedHistory = new List<int>();

            TorsoTrackedHistory = new List<int>();

        }

        public List<TimeSpan> Timestamps { get; private set; }

        public List<int> HandsTrackedHistory { get; private set; }

        public List<int> HeadTrackedHistory { get; private set; }

        public List<int> TorsoTrackedHistory { get; private set; }

        public static float HeadsTrackedAmount { get; private set; }

        public static float TorsosTrackedAmount { get; private set; }

        public void AddNewBodyFrame(List<Body> bodies, TimeSpan timestamp)
        {
            int headsTracked = 0;
            int handsTracked = 0;
            int torsosTracked = 0;
            foreach (var body in bodies)
            {
                headsTracked++;
                foreach (var hand in body.Hands)
                {
                    if (hand != null)
                    {
                        handsTracked++;
                    }

                }
                if (body.Torso != null)
                {
                    torsosTracked++;
                }
            }

            Timestamps.Add(timestamp);
            HeadTrackedHistory.Add(headsTracked);
            HandsTrackedHistory.Add(handsTracked);
            TorsoTrackedHistory.Add(torsosTracked);
        }

        /// <summary>
        /// Calculates the average amount of tracked heads, torsos and hands
        /// </summary>
        public void CalculateTrackingSuccess()
        {
            int totalHandsTracked = 0;
            int totalHeadsTracked = 0;
            int totalTorsosTracked = 0;
            int numberOfFrames = HeadTrackedHistory.Count;

            for (int i = 0; i < numberOfFrames; i++)
            {
                totalHeadsTracked += HeadTrackedHistory[i];
                totalHandsTracked += HandsTrackedHistory[i];
                totalTorsosTracked += TorsoTrackedHistory[i];
            }
                
            int totalNumberPossibleHands = NumberOfHands * totalHeadsTracked;

            HeadsTrackedAmount = (float) totalHeadsTracked / (float) numberOfFrames * NumberOfBodiesInFrame;
            TorsosTrackedAmount = (float)totalTorsosTracked / (float)numberOfFrames * NumberOfBodiesInFrame;
            float handsTrackedAmount = (float)totalHandsTracked / (float)totalNumberPossibleHands;

            Console.WriteLine("HeadsTrackedSuccess: {0}%", HeadsTrackedAmount * 100);
            Console.WriteLine("HandsTrackedSuccess: {0}%", handsTrackedAmount * 100);
            Console.WriteLine("TorsosTrackedSuccess: {0}%", TorsosTrackedAmount * 100);
        }
    }
}
