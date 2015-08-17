//
// Written by Leif Erik Bjoerkli
//


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
        public static List<int> CreateRegions(CameraSpacePoint[] cameraSpacePoints)
        {
            Stopwatch stopwatch = new Stopwatch();

            float[] integralImage = CreateStraightIntegralImage(cameraSpacePoints);

            var candidates = new List<int>();

            stopwatch.Restart();

            candidates.AddRange(SlideWindow(integralImage,GlobVar.HaarOuterWindowSize,GlobVar.HaarInnerWindowSize,5f,Thresholds.HaarDifferenceBetweenInnerAndOuterRectangle));

            DisjointSet disjointSet = GroupPoints3D(candidates);

            List<int> filteredCandidates = FilterPoints3D(disjointSet);
            
            return filteredCandidates;
        }


        private static List<Point> SlideTripleWindow(float[] integralImage, int outerWindowSize, int innerWindowWidth, int smallestWindowSize, float maxThreshold, float minThreshold, float maxThreshold2, float minThreshold2)
        {
            int smallestWindowMargin = (innerWindowWidth - smallestWindowSize) / 2;

            List<Point> candidates = new List<Point>();

            int marginInnerWindow = (outerWindowSize - innerWindowWidth) / 2;
            int innerWindowArea = innerWindowWidth * innerWindowWidth;
            int smallestWindowArea = smallestWindowSize * smallestWindowSize;

            for (int i = 0; i < GlobVar.ScaledFrameHeight - innerWindowWidth; i += 2)
            {
                for (int j = 0; j < GlobVar.ScaledFrameWidth - innerWindowWidth; j += 2)
                {
                    int outerWindowHeight = outerWindowSize;
                    int outerWindowWidth = outerWindowSize;

                    int iOuter = i - marginInnerWindow;
                    int jOuter = j - marginInnerWindow;

                    if (iOuter < 0)
                    {
                        outerWindowHeight += iOuter;
                        iOuter = 0;
                    }
                    if (jOuter < 0)
                    {
                        outerWindowWidth += jOuter;
                        jOuter = 0;
                    }
                    if (iOuter + outerWindowSize > GlobVar.ScaledFrameHeight - 1)
                    {
                        outerWindowHeight += iOuter + outerWindowSize - GlobVar.ScaledFrameHeight - 1;
                        iOuter = GlobVar.ScaledFrameHeight - 1 - outerWindowHeight;
                    }
                    if (jOuter + outerWindowSize > GlobVar.ScaledFrameWidth - 1)
                    {
                        outerWindowWidth += jOuter + outerWindowSize - GlobVar.ScaledFrameWidth - 1;
                        jOuter = GlobVar.ScaledFrameWidth - 1 - outerWindowWidth;
                    }

                    int innerIndexA = GlobUtils.GetIndex(j, i);
                    int innerIndexB = GlobUtils.GetIndex(j + innerWindowWidth, i);
                    int innerIndexC = GlobUtils.GetIndex(j, i + innerWindowWidth);
                    int innerIndexD = GlobUtils.GetIndex(j + innerWindowWidth, i + innerWindowWidth);

                    int outerIndexA = GlobUtils.GetIndex(jOuter, iOuter);
                    int outerIndexB = GlobUtils.GetIndex(jOuter + outerWindowWidth, iOuter);
                    int outerIndexC = GlobUtils.GetIndex(jOuter, iOuter + outerWindowHeight);
                    int outerIndexD = GlobUtils.GetIndex(jOuter + outerWindowWidth, iOuter + outerWindowHeight);

                    int smallestIndexA = GlobUtils.GetIndex(j + smallestWindowMargin, i + smallestWindowMargin);
                    int smallestIndexB = GlobUtils.GetIndex(j + innerWindowWidth - smallestWindowMargin, i + smallestWindowMargin);
                    int smallestIndexC = GlobUtils.GetIndex(j + smallestWindowMargin, i + innerWindowWidth - smallestWindowMargin);
                    int smallestIndexD = GlobUtils.GetIndex(j + innerWindowWidth - smallestWindowMargin, i + innerWindowWidth - smallestWindowMargin);

                    int outerWindowArea = GlobUtils.CalculatePixelAreaFromIndexes(outerIndexA, outerIndexB, outerIndexC);

                    float sumSmallestWindow = (integralImage[smallestIndexA] + integralImage[smallestIndexD] -
                                            integralImage[smallestIndexB] - integralImage[smallestIndexC]);
                    float averageSmallestWindow = sumSmallestWindow / smallestWindowArea;

                    if (averageSmallestWindow > GlobVar.MaxDepthMeter - Thresholds.DetectionHeightHeadMin)
                    {
                        continue;
                    }


                    float sumInnerWindow = (integralImage[innerIndexA] + integralImage[innerIndexD] -
                                            integralImage[innerIndexB] - integralImage[innerIndexC]);
                    float sumOuterWindow = (integralImage[outerIndexA] + integralImage[outerIndexD] -
                                            integralImage[outerIndexB] - integralImage[outerIndexC]);
                    float averageInnerWindow = sumInnerWindow / innerWindowArea;

                    if (averageInnerWindow > GlobVar.MaxDepthMeter - Thresholds.DetectionHeightHeadMin)
                    {
                        continue;
                    }
                    float averageOuterWindow = (sumOuterWindow - sumInnerWindow) / (outerWindowArea - innerWindowArea);

                    //Console.WriteLine(averageInnerWindow - averageOuterWindow);
                    //Console.WriteLine(averageSmallestWindow - averageInnerWindow);
                    if ((averageOuterWindow - averageInnerWindow) > minThreshold && (averageOuterWindow - averageInnerWindow) < maxThreshold && (averageInnerWindow - averageSmallestWindow) > minThreshold2 && (averageInnerWindow - averageSmallestWindow) < maxThreshold2)
                    {
                        //Console.WriteLine(averageInnerWindow - averageOuterWindow);
                        var candidate = GlobUtils.GetMidPointFromIndexes(innerIndexB, innerIndexC);
                        candidates.Add(candidate);
                        if (Diagnostics.ShowHaarRects)
                        {
                            GraphicsUtils.DrawRectangle(new IndexRectangle(smallestIndexA, smallestIndexB, smallestIndexC));
                            //Graphics.DrawRectangle(new
                            //IndexRectangle(outerIndexA, outerIndexB,
                            //outerIndexC));
                        }
                    }
                }
            }
            return candidates;
        }

        private static List<int> SlideWindow(float[] integralImage, int outerWindowSize, int innerWindowWidth, float maxThreshold, float minThreshold)
        {
            var candidates = new List<int>();

            int marginInnerWindow = (outerWindowSize - innerWindowWidth) / 2;
            int innerWindowArea = innerWindowWidth*innerWindowWidth;

            for (int i = 0; i < GlobVar.ScaledFrameHeight - innerWindowWidth; i += 2)
            {
                for (int j = 0; j < GlobVar.ScaledFrameWidth - innerWindowWidth; j += 2)
                {
                    int outerWindowHeight = outerWindowSize;
                    int outerWindowWidth = outerWindowSize;

                    int iOuter = i - marginInnerWindow;
                    int jOuter = j - marginInnerWindow;

                    if (iOuter < 0)
                    {
                        outerWindowHeight += iOuter;     
                        iOuter = 0;
                    }
                    if (jOuter < 0)
                    {
                        outerWindowWidth += jOuter;
                        jOuter = 0;
                    }
                    if (iOuter + outerWindowSize > GlobVar.ScaledFrameHeight - 1)
                    {
                        outerWindowHeight += iOuter + outerWindowSize - GlobVar.ScaledFrameHeight - 1;
                        iOuter = GlobVar.ScaledFrameHeight - 1 - outerWindowHeight;
                    }
                    if (jOuter + outerWindowSize > GlobVar.ScaledFrameWidth - 1)
                    {
                        outerWindowWidth += jOuter + outerWindowSize - GlobVar.ScaledFrameWidth - 1;
                        jOuter = GlobVar.ScaledFrameWidth - 1 - outerWindowWidth;
                    }

                    int innerIndexA = GlobUtils.GetIndex(j, i);
                    int innerIndexB = GlobUtils.GetIndex(j + innerWindowWidth, i);
                    int innerIndexC = GlobUtils.GetIndex(j, i + innerWindowWidth);
                    int innerIndexD = GlobUtils.GetIndex(j + innerWindowWidth, i + innerWindowWidth);

                    int outerIndexA = GlobUtils.GetIndex(jOuter, iOuter);
                    int outerIndexB = GlobUtils.GetIndex(jOuter + outerWindowWidth, iOuter);
                    int outerIndexC = GlobUtils.GetIndex(jOuter, iOuter + outerWindowHeight);
                    int outerIndexD = GlobUtils.GetIndex(jOuter + outerWindowWidth, iOuter + outerWindowHeight);

                    int outerWindowArea = GlobUtils.CalculatePixelAreaFromIndexes(outerIndexA, outerIndexB, outerIndexC);

                    float sumInnerWindow = (integralImage[innerIndexA] + integralImage[innerIndexD] -
                                            integralImage[innerIndexB] - integralImage[innerIndexC]);
                    float sumOuterWindow = (integralImage[outerIndexA] + integralImage[outerIndexD] -
                                            integralImage[outerIndexB] - integralImage[outerIndexC]);
                    float averageInnerWindow = sumInnerWindow / innerWindowArea;

                    if (averageInnerWindow > GlobVar.MaxDepthMeter - Thresholds.DetectionHeightHeadMin)
                    {
                        continue;
                    }
                    float averageOuterWindow = (sumOuterWindow - sumInnerWindow) / (outerWindowArea - innerWindowArea);

                    //Console.WriteLine(averageOuterWindow - averageInnerWindow);

                    if ((averageOuterWindow - averageInnerWindow) > minThreshold && (averageOuterWindow - averageInnerWindow) < maxThreshold)
                    {
                        //Console.WriteLine(averageOuterWindow - averageInnerWindow);
                        int candidate = GlobUtils.GetHighestValidPointIndexInRectangle(innerIndexB, innerIndexC);
                        candidates.Add(candidate);
                        if (Diagnostics.ShowHaarRects)
                        {
                            GraphicsUtils.DrawRectangle(new IndexRectangle(innerIndexA, innerIndexB, innerIndexC));
                            //Graphics.DrawRectangle(new
                            //IndexRectangle(outerIndexA, outerIndexB,
                            //outerIndexC));
                        }

                        var v = GlobVar.SubtractedFilteredPointCloud[candidate];
                    }
                }
            }
            return candidates;
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

        private static DisjointSet GroupPoints3D(List<int> candidatePoints)
        {
            DisjointSet disjointSet = new DisjointSet(candidatePoints);

            for (int i = 0; i < candidatePoints.Count; i++)
            {
                for (int j = 0; j < candidatePoints.Count; j++)
                {
                    if (i == j) { continue; };
                    //Console.WriteLine(GlobUtils.GetDistanceInFrame(candidatePoints[i], candidatePoints[j]));
                    if (GlobUtils.GetEuclideanDistanceXYPlane(candidatePoints[i], candidatePoints[j]) < Thresholds.HaarRectangleMaxDistance)
                    {
                        disjointSet.Union(i, j);
                    }
                }
            }

            return disjointSet;
        }

        private static List<int> FilterPoints(DisjointSet disjointSet)
        {
            List<int> filteredCandidatesIndex = new List<int>();

            for (int i = 0; i < disjointSet.Count; i++)
            {
                int setRepresentative = disjointSet.Find(i);
                if (!filteredCandidatesIndex.Contains(setRepresentative) && disjointSet.SetSize(setRepresentative)>Thresholds.HaarRectangleMinCount)
                { 
                    filteredCandidatesIndex.Add(setRepresentative);
                }
            }


            return filteredCandidatesIndex;
        }


        // Basically only filter for amount of rectangles
        private static List<int> FilterPoints3D(DisjointSet disjointSet)
        {

            var representatives = new List<int>();

            for (int i = 0; i < disjointSet.Count; i++)
            {
                int setRepresentative = disjointSet.Find(i);
                if (!representatives.Contains(setRepresentative) && disjointSet.SetSize(setRepresentative) > Thresholds.HaarRectangleMinCount)
                {
                    representatives.Add(setRepresentative);
                }
            }

            var highestCandidates = new List<int>();

            foreach (var representative in representatives)
            {
                highestCandidates.Add(disjointSet.HighestIndexInTree[representative]);
            }

            return highestCandidates;
        }
        
    }
}
