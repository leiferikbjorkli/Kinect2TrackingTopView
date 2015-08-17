using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;

namespace InteractionDetection
{
    class HeatMap
    {
        private Dictionary<int, int>[] map;
        private byte[] bgraMap;
        private int maxIntensity;
        
        public HeatMap()
        {
            map = new Dictionary<int, int>[GlobVar.ScaledFrameLength];

            for (int i = 0; i < map.Length; i++)
            {
                map[i] = new Dictionary<int, int>();
            }

            bgraMap = new byte[GlobVar.ScaledFrameLength*4];
            maxIntensity = 0;

        }

        public void AddFrame(int[] heatFrame)
        {
            for (int i = 0; i < heatFrame.Length; i++)
            {
                int pixelBodyId = heatFrame[i];

                if (pixelBodyId != 0)
                {
                    if (map[i].ContainsKey(pixelBodyId))
                    {
                        map[i][pixelBodyId]++;
                        if (map[i][pixelBodyId] > maxIntensity)
                        {
                            maxIntensity = map[i][pixelBodyId];
                        }
                    }
                    else
                    {
                        map[i].Add(pixelBodyId,1);
                    }
                }
            }
            ClearHeatCanvas();
        }

        private void ClearHeatCanvas()
        {
            GlobVar.HeatCanvas = new int[GlobVar.ScaledFrameLength];
        }

        public byte[] GetMap()
        {
            FilterNoise();
            NormalizeMap();
            CreateRgbaMap();

            return bgraMap;
        }

        private void NormalizeMap()
        {
            for (int i = 0; i < map.Length; i++)
            {
                var newDict = new Dictionary<int,int>();
                foreach (var element in map[i])
                {
                    newDict.Add(element.Key, (int)(((double)element.Value / (double)maxIntensity) * 255));
                }
                map[i] = newDict;
            }
        }

        public void CreateRgbaMap()
        {
            for (int i = 0; i < map.Length; i++)
            {
                int rgbaHeatMapIndex = i * 4;

                var color = GetColorFromHeatData(map[i], i);

                double totalIntensity = 0;
                foreach (var element in map[i])
                {
                    totalIntensity += element.Value;
                }

                bgraMap[rgbaHeatMapIndex] = color[0];
                bgraMap[rgbaHeatMapIndex + 1] = color[1];
                bgraMap[rgbaHeatMapIndex + 2] = color[2];
                bgraMap[rgbaHeatMapIndex + 3] = color[3];

            }
        }

        private byte[] GetColorFromHeatData(Dictionary<int,int> heatDictionary,int i)
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

        private void FilterNoise()
        {
            int topBodyId = GetMostTrackedBodyId();

            for (int i = 0; i < map.Length; i++)
            {
                var newDict = new Dictionary<int, int>();
                foreach (var element in map[i])
                {
                    if (element.Key == topBodyId)
                    {
                        newDict.Add(1,element.Value);
                    }
                }
                map[i] = newDict;
            }
        }

        private int GetMostTrackedBodyId()
        {

            var count = new Dictionary<int,int>();

            for (int i = 1; i <= GlobVar.MaxBodyCount; i++)
            {
                count.Add(i,0);
            }

            foreach (var index in map)
            {
                foreach (var heat in index)
                {
                    count[heat.Key] += heat.Value;
                }
            }

            var sortedCount = count.OrderByDescending(kvp => kvp.Value);

            return sortedCount.ElementAt(0).Key;
        }
    }
}
