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
        public static Point[] CreateRegions(CameraSpacePoint[] cameraSpacePoints)
        {
            // try with 3 sizes

            float[] integralImage = CreateStraightIntegralImage(cameraSpacePoints);

            List<Point> candidates = new List<Point>();

            Stopwatch sw = new Stopwatch();
            sw.Start();
            SlideWindow(50, 30, integralImage,candidates);
            SlideWindow(25, 15, integralImage, candidates);

            sw.Stop();
            //Console.WriteLine("Slidewindow, {0}",sw.ElapsedMilliseconds);

            sw.Start();
            DisjointSet disjointSet = GroupPoints(candidates);
            sw.Stop();
            //Console.WriteLine("Disjoint, {0}",sw.ElapsedMilliseconds);

            sw.Start();
            Point[] filteredCandidates = FilterPoints(disjointSet,candidates);
            sw.Stop();
            //Console.WriteLine("FilterNew, {0}", sw.ElapsedMilliseconds);

            return filteredCandidates;
        }

        private static void SlideWindow(int outerWindowWidth, int innerWindowWidth, float[] integralImage,
            List<Point> candidates)
        {

            int marginInnerWindow = (outerWindowWidth - innerWindowWidth)/2;
            int innerWindowArea = innerWindowWidth*innerWindowWidth;

            for (int i = 0; i < GlobVar.ScaledFrameHeight - innerWindowWidth; i += 3)
            {
                for (int j = 0; j < GlobVar.ScaledFrameWidth - innerWindowWidth; j += 3)
                {
                    int iOuter = i - marginInnerWindow;
                    int jOuter = j - marginInnerWindow;

                    if (iOuter < 0)
                    {
                        iOuter = 0;
                    }
                    if (jOuter < 0)
                    {
                        jOuter = 0;
                    }
                    if (iOuter + outerWindowWidth > GlobVar.ScaledFrameHeight - 1)
                    {
                        iOuter = GlobVar.ScaledFrameHeight - 1 - outerWindowWidth;
                    }
                    if (jOuter + outerWindowWidth > GlobVar.ScaledFrameWidth - 1)
                    {
                        jOuter = GlobVar.ScaledFrameWidth - 1 - outerWindowWidth;
                    }

                    int innerIndexA = GlobUtils.GetIndex(j, i);
                    int innerIndexB = GlobUtils.GetIndex(j + innerWindowWidth, i);
                    int innerIndexC = GlobUtils.GetIndex(j, i + innerWindowWidth);
                    int innerIndexD = GlobUtils.GetIndex(j + innerWindowWidth, i + innerWindowWidth);

                    int outerIndexA = GlobUtils.GetIndex(jOuter, iOuter);
                    int outerIndexB = GlobUtils.GetIndex(jOuter + outerWindowWidth, iOuter);
                    int outerIndexC = GlobUtils.GetIndex(jOuter, iOuter + outerWindowWidth);
                    int outerIndexD = GlobUtils.GetIndex(jOuter + outerWindowWidth, iOuter + outerWindowWidth);

                    int outerWindowArea = GlobUtils.CalculatePixelAreaFromIndexes(outerIndexA, outerIndexB, outerIndexC);

                    float sumInnerWindow = (integralImage[innerIndexA] + integralImage[innerIndexD] -
                                            integralImage[innerIndexB] - integralImage[innerIndexC]);
                    float sumOuterWindow = (integralImage[outerIndexA] + integralImage[outerIndexD] -
                                            integralImage[outerIndexB] - integralImage[outerIndexC]);
                    float averageInnerWindow = sumInnerWindow/innerWindowArea;
                    float averageOuterWindow = (sumOuterWindow - sumInnerWindow)/(outerWindowArea - innerWindowArea);
                    
                    //Console.WriteLine(averageInnerWindow - averageOuterWindow);

                    if ((averageInnerWindow - averageOuterWindow) < Thresholds.SlidingWindow)
                    {
                        //Console.WriteLine(averageInnerWindow - averageOuterWindow);
                        var candidate = GlobUtils.GetMidPointFromIndexes(innerIndexB, innerIndexC);
                        candidates.Add(candidate);
                        //Graphics.DrawRectangle(new IndexRectangle(innerIndexA, innerIndexB, innerIndexC));
                    }
                }
            }
        }

        private static float[] CreateStraightIntegralImage(CameraSpacePoint[] cameraSpacePoints) 
        {
            float[] integralImage = new float[GlobVar.ScaledFrameLength];
            for (var i = 0; i < GlobVar.ScaledFrameHeight; i++)
            {
                for (var j = 0; j < GlobVar.ScaledFrameWidth; j++)
                {
                    int indexCurrent = i * GlobVar.ScaledFrameWidth + j;
                    int indexUpLeft = (i - 1) * GlobVar.ScaledFrameWidth + j - 1;
                    int indexUp = (i - 1) * GlobVar.ScaledFrameWidth + j;
                    int indexLeft = i * GlobVar.ScaledFrameWidth + j - 1;

                    if (i==0 && j==0)
                    {
                        integralImage[indexCurrent] = cameraSpacePoints[indexCurrent].Z;
                    }
                    else if (i==0)
                    {
                        integralImage[indexCurrent] = cameraSpacePoints[indexCurrent].Z + integralImage[indexLeft];
                    }
                    else if (j == 0)
                    {
                        integralImage[indexCurrent] = cameraSpacePoints[indexCurrent].Z + integralImage[indexUp];
                    }
                    else 
                    {
                        integralImage[indexCurrent] = cameraSpacePoints[indexCurrent].Z + integralImage[indexUp] + integralImage[indexLeft] - integralImage[indexUpLeft];
                    }
                }
            }
            return integralImage;
        }

        private static DisjointSet GroupPoints(List<Point> candidatePoints)
        {

            DisjointSet disjointSet = new DisjointSet(candidatePoints);
            for (int i = 0; i < candidatePoints.Count; i++)
            {
                for (int j = 0; j < candidatePoints.Count; j++)
                {
                    if (i == j) { continue; };
                    //Console.WriteLine(GlobUtils.GetDistanceInFrame(candidatePoints[i], candidatePoints[j]));
                    if (GlobUtils.GetDistanceInFrame(candidatePoints[i], candidatePoints[j]) < Thresholds.HaarRectangleMaxDistance)
                    {
                        disjointSet.Union(i,j);
                    }
                }
            }
            return disjointSet;
        }

        private static Point[] FilterPoints(DisjointSet disjointSet,List<Point> candidates)
        {
            List<int> filteredCandidatesIndex = new List<int>();

            for (int i = 0; i < disjointSet.Count; i++)
            {
                int setRepresentative = disjointSet.Find(i);
                if (!filteredCandidatesIndex.Contains(setRepresentative) && disjointSet.SetSize(setRepresentative)>Thresholds.HaarRectangleCount)
                { 
                    filteredCandidatesIndex.Add(setRepresentative);
                }
            }

            Point[] filteredCandidates = new Point[filteredCandidatesIndex.Count];
            for (int i = 0; i < filteredCandidates.Length; i++)
            {
                filteredCandidates[i] = new Point((int)disjointSet.AverageX[filteredCandidatesIndex[i]], (int)disjointSet.AverageY[filteredCandidatesIndex[i]]);
            }

            return filteredCandidates;
        }

        //private static void GroupRectangles(List<Point> candidates,
        //Dictionary<Point, int> candidatesPopularity) {


        //    for (int i = 0; i < candidates.Count; i++)
        //    {
        //        candidatesPopularity.Add(candidates[i], 0);
        //    }

        //    for (int i = 0; i < candidates.Count; i++)
        //    {
        //        for (int j = 0; j < candidates.Count; j++)
        //        {
        //            if(i==j) {continue;}
        //            //Console.WriteLine(GlobUtils.GetDistanceInFrame(candidates[j], candidates[i]));
        //            if (GlobUtils.GetDistanceInFrame(candidates[j], candidates[i]) < Thresholds.Grouping)
        //            {
        //                candidatesPopularity[candidates[i]] += 1;
        //            }
        //        }
        //    }
        //}

        //private static List<Point> FilterPopularRectangles(Dictionary<Point, int> candidatesPopularity)
        //{
        //    int i = 0;
        //    int j = 1;

        //    while (i < candidatesPopularity.Count)
        //    {
        //        if (i != j)
        //        {
        //            Point firstPoint = candidatesPopularity.ElementAt(i).Key;
        //            Point secondPoint = candidatesPopularity.ElementAt(j).Key;

        //            if (GlobUtils.GetDistanceInFrame(firstPoint, secondPoint) < Thresholds.FilterCandidateDistance)
        //            {
        //                if (candidatesPopularity.ElementAt(i).Value > candidatesPopularity.ElementAt(j).Value)
        //                {
        //                    candidatesPopularity.Remove(secondPoint);
        //                    if (i > j)
        //                    {
        //                        i--;
        //                    }
        //                }
        //                else
        //                {
        //                    candidatesPopularity.Remove(firstPoint);
        //                    if (i < j)
        //                    {
        //                        j--;
        //                    }
        //                }
        //            }
        //            else
        //            {
        //                j++;
        //            }
        //        }
        //        else
        //        {
        //            j++;
        //        }

        //        if(j == candidatesPopularity.Count)
        //        {
        //            j = 0;
        //            i++;
        //        }
        //    }
        //    return candidatesPopularity.Keys.ToList();
        //}
    }
}
