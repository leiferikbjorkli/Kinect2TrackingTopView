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
    class EnergyHistory
    {
        private static List<Dictionary<int, double>> _energyHistory;

        public EnergyHistory()
        {
            _energyHistory = new List<Dictionary<int, double>>();
        }

        public List<Dictionary<int, double>> Get()
        {
            var filteredHistory = FilterMostTrackedBodyId(_energyHistory);
            var smoothedHistory = MedianOneDimension(filteredHistory);

            return smoothedHistory;
        }

        /// <summary>
        /// If only one body is supposed to be tracked, this method filters away all other bodies than the one that is tracked the most.
        /// </summary>
        private List<Dictionary<int, double>> FilterMostTrackedBodyId(List<Dictionary<int, double>> energyHistory)
        {
            var mostTrackedBodyId = GetMostTrackedBodyIdFromEnergyHistory(energyHistory);

            var filteredEnergyHistory = new List<Dictionary<int,double>>();

            foreach (var frame in energyHistory)
            {
                var filteredFrame = new Dictionary<int,double>();
                foreach (var energy in frame)
                {
                    if (energy.Key == mostTrackedBodyId)
                    {
                        filteredFrame.Add(1,energy.Value);
                    }
                }
                filteredEnergyHistory.Add(filteredFrame);
            }

            return filteredEnergyHistory;
        }

        private static int GetMostTrackedBodyIdFromEnergyHistory(List<Dictionary<int, double>> energyHistory)
        {
            var count = new Dictionary<int, int>();

            for (int i = 1; i <= GlobVar.MaxBodyCount; i++)
            {
                count.Add(i, 0);
            }

            foreach (var frame in energyHistory)
            {
                foreach (var energy in frame)
                {
                    count[energy.Key]++;
                }
            }

            var sortedCount = count.OrderByDescending(kvp => kvp.Value);

            return sortedCount.ElementAt(0).Key;
        }

        /// <summary>
        /// One dimensional median filter that smoothes away energy spikes that could occur from occasional lost tracking.
        /// </summary>
        private static List<Dictionary<int, double>> MedianOneDimension(List<Dictionary<int, double>> energyHistory)
        {
            var smoothedEnergyHistory = new List<Dictionary<int, double>>();

            smoothedEnergyHistory.Add(new Dictionary<int, double>());
            smoothedEnergyHistory.Add(new Dictionary<int, double>());
            for (int i = 2; i < energyHistory.Count-2; i++)
            {
                var filteredFrame = new Dictionary<int,double>();

                foreach (var frameEnergy in energyHistory[i])
                {
                    var energyKernel = new double[5];

                    int bodyId = frameEnergy.Key;

                    double energyFrame1 = 0;
                    energyHistory[i - 2].TryGetValue(bodyId, out energyFrame1);
                    energyKernel[0] = energyFrame1;

                    double energyFrame2 = 0;
                    energyHistory[i - 1].TryGetValue(bodyId,out energyFrame2);
                    energyKernel[1] = energyFrame2;

                    double energyFrame3 = 0;
                    energyHistory[i].TryGetValue(bodyId, out energyFrame3);
                    energyKernel[2] = energyFrame3;

                    double energyFrame4 = 0;
                    energyHistory[i + 1].TryGetValue(bodyId, out energyFrame4);
                    energyKernel[3] = energyFrame4;

                    double energyFrame5 = 0;
                    energyHistory[i + 2].TryGetValue(bodyId, out energyFrame5);
                    energyKernel[4] = energyFrame5;

                    Array.Sort(energyKernel);
                    filteredFrame.Add(bodyId,energyKernel[2]);
                }
                smoothedEnergyHistory.Add(filteredFrame);
            }
            smoothedEnergyHistory.Add(new Dictionary<int, double>());
            smoothedEnergyHistory.Add(new Dictionary<int, double>());

            return smoothedEnergyHistory;
        }

        public static void Update(List<Body> bodies)
        {
            var currentFrameEnergy = new Dictionary<int,double>();
            foreach (var body in bodies)
            {
                currentFrameEnergy.Add(body.Id,body.EnergyLevel);
            }

            _energyHistory.Add(currentFrameEnergy);
        }

        /// <summary>
        /// Calculates the total timespan where a body was tracked and energy-use was calculated.
        /// </summary>
        private static TimeSpan CalculateTotalTimeWithTracking()
        {
            var totalTime = new TimeSpan();
            var hasTracking = false;
            var lastTimeStamp = new TimeSpan();

            for (int i = 0; i < _energyHistory.Count; i++)
            {
                if (_energyHistory[i].Count > 0)
                {
                    var currentTimeStamp = GlobVar.TimeStamps[i];
                    if (hasTracking)
                    {
                        var trackingTimeSpan = currentTimeStamp.Subtract(lastTimeStamp);
                        totalTime += trackingTimeSpan;
                        lastTimeStamp = currentTimeStamp;
                    }
                    else
                    {
                        lastTimeStamp = currentTimeStamp;
                        hasTracking = true;
                    }
                }
                else
                {
                    hasTracking = false;
                }
            }
            return totalTime;
        }
         
        /// <summary>
        /// Calculates the average Joule/s used.
        /// </summary>
        public static double GetAverageEnergyUsed()
        {
            double totalEnergy = 0;

            var totalTimeWithTracking = CalculateTotalTimeWithTracking();

            foreach (var frame in _energyHistory)
            {
                if (frame.Count > 0)
                {
                    foreach (var energy in frame)
                    {
                        totalEnergy += energy.Value;
                    }
                }
            }
            return totalEnergy / totalTimeWithTracking.TotalSeconds;
        }
    }
}
