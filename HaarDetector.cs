using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;
using System.Diagnostics;

namespace InteractionDetection
{
    static class HaarDetector
    {
        public static List<Point> CreateRegions(float[] depthMap)
        {
            // try with 3 sizes

            float[] integralImage = CreateStraightIntegralImage(depthMap);

            var candidatesPopularity = new Dictionary<Point, int>();
            List<Point> candidates = new List<Point>();
            SlideWindow(50, 30, integralImage,candidates);
            SlideWindow(25, 15, integralImage, candidates);
            

            GroupRectangles(candidates, candidatesPopularity);
            List<Point> filteredCandidates = FilterPopularRectangles(candidatesPopularity);

            return filteredCandidates;
        }

        private static void SlideWindow(int outerWindowWidth, int innerWindowWidth, float[] integralImage,List<Point> candidates )
        {
            int marginInnerWindow = (outerWindowWidth - innerWindowWidth)/2;
            int innerWindowArea = innerWindowWidth * innerWindowWidth;


            for (int i = 0; i < GlobVar.scaledFrameHeight - innerWindowWidth; i += 3)
            {
                for (int j = 0; j < GlobVar.scaledFrameWidth - innerWindowWidth; j += 3)
                {
                    int iOuter = i - marginInnerWindow;
                    int jOuter = j - marginInnerWindow;

                    if (iOuter < 0)
                    {
                        iOuter = 0;
                    }
                    if(jOuter < 0)
                    {
                        jOuter = 0;
                    }
                    if (iOuter + outerWindowWidth > GlobVar.scaledFrameHeight - 1)
                    {
                        iOuter = GlobVar.scaledFrameHeight - 1 - outerWindowWidth;
                    }
                    if (jOuter+outerWindowWidth > GlobVar.scaledFrameWidth-1)
                    {
                        jOuter = GlobVar.scaledFrameWidth - 1 - outerWindowWidth;
                    }

                    int innerIndexA = GlobUtils.GetIndex(j,i);
                    int innerIndexB = GlobUtils.GetIndex(j+innerWindowWidth,i);
                    int innerIndexC = GlobUtils.GetIndex(j,i+innerWindowWidth);
                    int innerIndexD = GlobUtils.GetIndex(j+innerWindowWidth,i+innerWindowWidth);

                    int outerIndexA = GlobUtils.GetIndex(jOuter,iOuter);
                    int outerIndexB = GlobUtils.GetIndex(jOuter+outerWindowWidth,iOuter);
                    int outerIndexC = GlobUtils.GetIndex(jOuter, iOuter + outerWindowWidth);
                    int outerIndexD = GlobUtils.GetIndex(jOuter+outerWindowWidth,iOuter+outerWindowWidth);

                    int outerWindowArea = GlobUtils.CalculatePixelAreaFromIndexes(outerIndexA,outerIndexB,outerIndexC);

                    float sumInnerWindow = (integralImage[innerIndexA] + integralImage[innerIndexD] - integralImage[innerIndexB] - integralImage[innerIndexC]);
                    float sumOuterWindow = (integralImage[outerIndexA] + integralImage[outerIndexD] - integralImage[outerIndexB] - integralImage[outerIndexC]);
                    float averageInnerWindow = sumInnerWindow / innerWindowArea;
                    float averageOuterWindow = (sumOuterWindow - sumInnerWindow) / (outerWindowArea - innerWindowArea);
                    //Console.WriteLine(averageInnerWindow - averageOuterWindow);

                    if ((averageInnerWindow-averageOuterWindow) > Thresholds.SlidingWindow)
                    {
                        //Console.WriteLine(averageInnerWindow - averageOuterWindow);
                        var candidate = GlobUtils.GetMidPointFromIndexes(innerIndexB,innerIndexC);
                        candidates.Add(candidate);

                        //Graphics.DrawRectangle(new IndexRectangle(innerIndexA,innerIndexB,innerIndexC));
                    }
                }   
            }
        }

        private static float[] CreateStraightIntegralImage(float[] depthMap) 
        {
            float[] integralImage = new float[GlobVar.scaledFrameLength];
            for (var i = 0; i < GlobVar.scaledFrameHeight; i++)
            {
                for (var j = 0; j < GlobVar.scaledFrameWidth; j++)
                {
                    int indexCurrent = i * GlobVar.scaledFrameWidth + j;
                    int indexUpLeft = (i - 1) * GlobVar.scaledFrameWidth + j - 1;
                    int indexUp = (i - 1) * GlobVar.scaledFrameWidth + j;
                    int indexLeft = i * GlobVar.scaledFrameWidth + j - 1;

                    if (i==0 && j==0)
                    {
                        integralImage[indexCurrent] = depthMap[indexCurrent];
                    }
                    else if (i==0)
                    {
                        integralImage[indexCurrent] = depthMap[indexCurrent] + integralImage[indexLeft];
                    }
                    else if (j == 0)
                    {
                        integralImage[indexCurrent] = depthMap[indexCurrent] + integralImage[indexUp];
                    }
                    else 
                    {
                        integralImage[indexCurrent] = depthMap[indexCurrent] + integralImage[indexUp] + integralImage[indexLeft] - integralImage[indexUpLeft];
                    }
                }
            }
            return integralImage;
        }

        private static void GroupRectangles(List<Point> candidates, Dictionary<Point, int> candidatesPopularity)
        {


            for (int i = 0; i < candidates.Count; i++)
            {
                candidatesPopularity.Add(candidates[i], 0);
            }

            for (int i = 0; i < candidates.Count; i++)
            {
                for (int j = 0; j < candidates.Count; j++)
                {
                    if(i==j) {continue;}
                    //Console.WriteLine(GlobUtils.GetDistanceInFrame(candidates[j], candidates[i]));
                    if (GlobUtils.GetDistanceInFrame(candidates[j], candidates[i]) < Thresholds.Grouping)
                    {
                        candidatesPopularity[candidates[i]] += 1;
                    }
                }
            }
        }

        private static List<Point> FilterPopularRectangles(Dictionary<Point, int> candidatesPopularity)
        {
            int i = 0;
            int j = 1;

            while (i < candidatesPopularity.Count)
            {
                if (i != j)
                {
                    Point firstPoint = candidatesPopularity.ElementAt(i).Key;
                    Point secondPoint = candidatesPopularity.ElementAt(j).Key;

                    if (GlobUtils.GetDistanceInFrame(firstPoint, secondPoint) < Thresholds.FilterCandidateDistance)
                    {
                        if (candidatesPopularity.ElementAt(i).Value > candidatesPopularity.ElementAt(j).Value)
                        {
                            candidatesPopularity.Remove(secondPoint);
                            if (i > j)
                            {
                                i--;
                            }
                        }
                        else
                        {
                            candidatesPopularity.Remove(firstPoint);
                            if (i < j)
                            {
                                j--;
                            }
                        }
                    }
                    else
                    {
                        j++;
                    }
                }
                else
                {
                    j++;
                }

                if(j == candidatesPopularity.Count)
                {
                    j = 0;
                    i++;
                }
            }
            return candidatesPopularity.Keys.ToList();
        }
    }
}
