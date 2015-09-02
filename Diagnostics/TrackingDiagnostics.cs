//
// Copyright (c) Leif Erik Bjoerkli, Norwegian University of Science and Technology, 2015.
// Distributed under the MIT License.
// (See accompanying file LICENSE or copy at http://opensource.org/licenses/MIT)
//  

using System;
using System.Collections.Generic;
using System.Linq;

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
            _headsTrackedHistory = new List<List<int>>();

            _torsosTrackedHistory = new List<List<int>>();

            _handsTrackedHistory = new List<Dictionary<int, int>>();

        }

        public static float HeadsTrackedAmount { get; private set; }

        public static float TorsosTrackedAmount { get; private set; }

        private static List<List<int>> _headsTrackedHistory;
        private static List<List<int>> _torsosTrackedHistory;
        private static List<Dictionary<int, int>> _handsTrackedHistory;

        private static int GetMostTrackedBodyIdFromTrackingHistory(List<List<int>> trackingHistory)
        {
            var count = new Dictionary<int, int>();

            for (int i = 1; i <= GlobVar.MaxBodyCount; i++)
            {
                count.Add(i, 0);
            }

            foreach (var frame in trackingHistory)
            {
                foreach (var detection in frame)
                {
                    count[detection]++;
                }
            }

            var sortedCount = count.OrderByDescending(kvp => kvp.Value);

            return sortedCount.ElementAt(0).Key;
        }

        public static void AddNewBodyFrame(List<Body> bodies)
        {
            var frameHeadTracking = new List<int>();
            var frameTorsoTracking = new List<int>();
            var frameHandTracking = new Dictionary<int, int>();

            foreach (var body in bodies)
            {
                frameHeadTracking.Add(body.Id);
                if (body.Torso != null)
                {
                    frameTorsoTracking.Add(body.Id);
                }
                var handCount = 0;
                foreach (var hand in body.Hands)
                {
                    if (hand != null)
                    {
                        handCount++;
                    }
                }
                frameHandTracking.Add(body.Id,handCount);
            }
            _headsTrackedHistory.Add(frameHeadTracking);
            _torsosTrackedHistory.Add(frameTorsoTracking);
            _handsTrackedHistory.Add(frameHandTracking);
        }

        /// <summary>
        /// Calculates the average amount of tracked heads, torsos and hands
        /// </summary>
        public static void CalculateTrackingSuccess()
        {
            int mostTrackedBody = GetMostTrackedBodyIdFromTrackingHistory(_headsTrackedHistory);

            int totalHandsTracked = 0;
            int totalHeadsTracked = 0;
            int totalTorsosTracked = 0;

            var numberOfFrames = _headsTrackedHistory.Count;

            for (int i = 0; i < numberOfFrames; i++)
            {
                foreach (var detection in _headsTrackedHistory[i])
                {
                    if (detection == mostTrackedBody)
                    {
                        totalHeadsTracked++;
                        if (_torsosTrackedHistory[i].Contains(mostTrackedBody))
                        {
                            totalTorsosTracked++;
                        }
                        if (_handsTrackedHistory[i].ContainsKey(mostTrackedBody))
                        {
                            totalHandsTracked += _handsTrackedHistory[i][mostTrackedBody];
                        }
                    }
                }
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
