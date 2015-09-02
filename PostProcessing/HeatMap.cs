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
using Microsoft.Kinect;

namespace Kinect2TrackingTopView
{
    class HeatMap
    {
        private readonly Dictionary<int, int>[] _map;
        private int _maxIntensity;
        
        public HeatMap()
        {
            _map = new Dictionary<int, int>[GlobVar.ScaledFrameLength];

            for (int i = 0; i < _map.Length; i++)
            {
                _map[i] = new Dictionary<int, int>();
            }
            _maxIntensity = 0;

        }

        public void AddFrame(int[] heatFrame)
        {
            for (int i = 0; i < heatFrame.Length; i++)
            {
                int pixelBodyId = heatFrame[i];

                if (pixelBodyId != 0)
                {
                    if (_map[i].ContainsKey(pixelBodyId))
                    {
                        _map[i][pixelBodyId]++;
                        if (_map[i][pixelBodyId] > _maxIntensity)
                        {
                            _maxIntensity = _map[i][pixelBodyId];
                        }
                    }
                    else
                    {
                        _map[i].Add(pixelBodyId,1);
                    }
                }
            }
            ClearHeatCanvas();
        }

        private static void ClearHeatCanvas()
        {
            GlobVar.HeatCanvas = new int[GlobVar.ScaledFrameLength];
        }

        public byte[] Get()
        {
            var filteredHeatMap = FilterMostTrackedBodyId(_map);
            var normalizedHeatMap = NormalizeIntensitiesInHeatMap(filteredHeatMap);
            var bgraMap = CreateBgraMapFromHeatMap(normalizedHeatMap);

            return bgraMap;
        }

        private Dictionary<int, int>[] NormalizeIntensitiesInHeatMap(Dictionary<int, int>[] filteredHeatMap)
        {
            var normalizedHeatMap = new Dictionary<int, int>[filteredHeatMap.Length];

            for (int i = 0; i < filteredHeatMap.Length; i++)
            {
                var newHeatFrame = new Dictionary<int, int>();
                foreach (var element in filteredHeatMap[i])
                {
                    newHeatFrame.Add(element.Key, (int)(((double)element.Value / (double)_maxIntensity) * 255));
                }
                normalizedHeatMap[i] = newHeatFrame;
            }

            return normalizedHeatMap;
        }

        public static byte[] CreateBgraMapFromHeatMap(Dictionary<int, int>[] normalizedHeatMap)
        {
            var bgraMap = new byte[GlobVar.ScaledFrameLength * 4];

            for (int i = 0; i < normalizedHeatMap.Length; i++)
            {
                int rgbaHeatMapIndex = i * 4;

                var color = GetColorFromHeatData(normalizedHeatMap[i], i);

                double totalIntensity = 0;
                foreach (var element in normalizedHeatMap[i])
                {
                    totalIntensity += element.Value;
                }

                bgraMap[rgbaHeatMapIndex] = color[0];
                bgraMap[rgbaHeatMapIndex + 1] = color[1];
                bgraMap[rgbaHeatMapIndex + 2] = color[2];
                bgraMap[rgbaHeatMapIndex + 3] = color[3];
            }
            return bgraMap;
        }

        private static byte[] GetColorFromHeatData(Dictionary<int,int> heatDictionary,int i)
        {
            int totalIntensity = 0;
            foreach (var element in heatDictionary)
            {
                totalIntensity += element.Value;
            }

            var color = new byte[4];
            foreach (var element in heatDictionary)
            {
                var idColor = GraphicsUtils.GetBgrColorFromBodyId(element.Key);
                var weight = (double)element.Value / (double)totalIntensity;

                color[0] += (byte)(idColor[0] * weight);
                color[1] += (byte)(idColor[1] * weight);
                color[2] += (byte)(idColor[2] * weight);
            }
                
            color[3] = (byte) totalIntensity;

            return color;
        }

        /// <summary>
        /// If only one body is supposed to be tracked, this method filters away all other bodies than the one that is tracked the most.
        /// </summary>
        private Dictionary<int, int>[] FilterMostTrackedBodyId(Dictionary<int, int>[] heatMap)
        {
            var filteredHeatMap = new Dictionary<int, int>[heatMap.Length];

            int mostTrackedBodyId = GetMostTrackedBodyIdFromHeatHistory();

            for (int i = 0; i < heatMap.Length; i++)
            {
                var newHeatFrame = new Dictionary<int, int>();
                foreach (var element in heatMap[i])
                {
                    if (element.Key == mostTrackedBodyId)
                    {
                        newHeatFrame.Add(1,element.Value);
                    }
                }
                filteredHeatMap[i] = newHeatFrame;
            }
            return filteredHeatMap;
        }

        private int GetMostTrackedBodyIdFromHeatHistory()
        {
            var bodyIdCount = new Dictionary<int,int>();

            for (int i = 1; i <= GlobVar.MaxBodyCount; i++)
            {
                bodyIdCount.Add(i,0);
            }

            foreach (var index in _map)
            {
                foreach (var heat in index)
                {
                    bodyIdCount[heat.Key] += heat.Value;
                }
            }

            var sortedCount = bodyIdCount.OrderByDescending(kvp => kvp.Value);

            return sortedCount.ElementAt(0).Key;
        }
    }
}
