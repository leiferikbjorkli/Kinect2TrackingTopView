using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace InteractionDetection
{
    static class ClassificationUtils
    {



        public static void ConnectedComponentLabeling(float[] depthMap,List<Point> candidatePoints )
        {
            int[] labelMap = new int[depthMap.Length];

            for (int i = 1; i < candidatePoints.Count; i++)
            {
                int candidateIndex = GetHighestIntensityInSurroundingArea(depthMap, candidatePoints[i]);

                Point candidate = GlobUtils.GetPoint(candidateIndex);
                int recursiveCount = 0;
                RecursiveLabeling(labelMap,depthMap,candidate,i+1,depthMap[candidateIndex],recursiveCount);
            }
            Console.WriteLine(candidatePoints.Count);
            for (int i = 0; i < labelMap.Length; i++)
            {
                if (labelMap[i] != 0)
                {
                    GlobVar.canvas[i] = (byte) 200;
                }
            }
        }

        private static int GetHighestIntensityInSurroundingArea(float[] depthMap, Point candidate)
        {
            const int searchPixelRange = 6;

            int xStartIndex = candidate.x - searchPixelRange;
            int yStartIndex = candidate.y - searchPixelRange;
            int xStopIndex = candidate.x + searchPixelRange;
            int yStopIndex = candidate.y + searchPixelRange;

            float maxIntensity = -1.0f;
            int maxPointIndex = -1;
            for (int i = yStartIndex; i < yStopIndex; i++)
            {
                for (int j = xStartIndex; j < xStopIndex; j++)
                {
                    int pixelIndex = GlobUtils.GetIndex(j, i);
                    float intensity = depthMap[pixelIndex];
                    if (intensity>maxIntensity)
                    {
                        maxIntensity = intensity;
                        maxPointIndex = pixelIndex;
                    }
                }
            }
            return maxPointIndex;
        }

        private static void RecursiveLabeling(int[] labelMap, float[] depthMap,Point pixel, int label, float previousDepth,int recursiveCount)
        {
            if(pixel.x > GlobVar.scaledFrameWidth-1 || pixel.x < 0 || pixel.y > GlobVar.scaledFrameHeight-1 || pixel.y < 0)
            {
                return;
            }

            int pixelIndex = GlobUtils.GetIndex(pixel);
            float pixelDepth = depthMap[pixelIndex];

            int x = pixel.x;
            int y = pixel.y;

            if (labelMap[pixelIndex] != label && Thresholds.Labeling>(Math.Abs(pixelDepth-previousDepth)))
            {
                labelMap[pixelIndex] = label;
                recursiveCount++;
                if (recursiveCount < Thresholds.RecursionDepth)
                {
                    RecursiveLabeling(labelMap, depthMap, new Point(x - 1, y), label, pixelDepth, recursiveCount);
                    RecursiveLabeling(labelMap, depthMap, new Point(x - 1, y - 1), label, pixelDepth, recursiveCount);
                    RecursiveLabeling(labelMap, depthMap, new Point(x, y - 1), label, pixelDepth, recursiveCount);
                    RecursiveLabeling(labelMap, depthMap, new Point(x + 1, y - 1), label, pixelDepth, recursiveCount);
                    RecursiveLabeling(labelMap, depthMap, new Point(x + 1, y), label, pixelDepth, recursiveCount);
                    RecursiveLabeling(labelMap, depthMap, new Point(x + 1, y + 1), label, pixelDepth, recursiveCount);
                    RecursiveLabeling(labelMap, depthMap, new Point(x, y + 1), label, pixelDepth, recursiveCount);
                    RecursiveLabeling(labelMap, depthMap, new Point(x - 1, y + 1), label, pixelDepth, recursiveCount);
                }
            }
        }
    }
}
