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
using System.Diagnostics;

namespace Kinect2TrackingTopView
{
    class HaarDetector
    {
        private readonly float[] _integralImage;

        public HaarDetector(CameraSpacePoint[] cameraSpacePoints)
        {
            _integralImage = CreateIntegralImage(cameraSpacePoints);
        }

        private static float[] CreateIntegralImage(CameraSpacePoint[] cameraSpacePoints) 
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

        public List<int> GetHeadCandidates()
        {
            List<int> possibleHeadCandidates = SlideWindow(GlobVar.HaarOuterWindowSize, GlobVar.HaarInnerWindowSize,
                Thresholds.HaarDifferenceBetweenInnerAndOuterRectangle);

            DisjointSet groupedHeadCandidates = GroupCandidates(possibleHeadCandidates);

            List<int> filteredHeadCandidates = FilterCandidates(groupedHeadCandidates);
            
            return filteredHeadCandidates;
        }

        /// <summary>
        /// Slides a window with a subwindow with sizes specified by the input parameters over the frame. The average depth in the windows are compared and if above a certain threshold considered to be head candidates.
        /// </summary>
        private List<int> SlideWindow(int outerWindowSize, int innerWindowWidth, float windowDifferenceThreshold)
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

                    int outerWindowArea = GlobUtils.CalculateRectangleAreaFromIndexes(outerIndexA, outerIndexB, outerIndexC);

                    float sumInnerWindow = (_integralImage[innerIndexA] + _integralImage[innerIndexD] -
                                            _integralImage[innerIndexB] - _integralImage[innerIndexC]);
                    float sumOuterWindow = (_integralImage[outerIndexA] + _integralImage[outerIndexD] -
                                            _integralImage[outerIndexB] - _integralImage[outerIndexC]);
                    float averageInnerWindow = sumInnerWindow / innerWindowArea;

                    if (averageInnerWindow > GlobVar.MaxSensingDepth - Thresholds.HaarDetectionHeightHeadMin)
                    {
                        continue;
                    }
                    float averageOuterWindow = (sumOuterWindow - sumInnerWindow) / (outerWindowArea - innerWindowArea);

                    if ((averageOuterWindow - averageInnerWindow) > windowDifferenceThreshold)
                    {
                        int candidate = GlobUtils.GetHighestValidPointIndexInRectangle(innerIndexB, innerIndexC);
                        candidates.Add(candidate);
                        if (Debugger.ShowHaarRects)
                        {
                            GraphicsUtils.DrawRectangle(new IndexRectangle(innerIndexA, innerIndexB, innerIndexC));
                        }
                    }
                }
            }
            return candidates;
        }

        /// <summary>
        /// Groups the candidates if the distance between them in the XY-plane is below threshold.
        /// </summary>
        private static DisjointSet GroupCandidates(List<int> candidatePoints)
        {
            DisjointSet groupedSet = new DisjointSet(candidatePoints);

            for (int i = 0; i < candidatePoints.Count; i++)
            {
                for (int j = 0; j < candidatePoints.Count; j++)
                {
                    if (i == j) { continue; };
                    if (GlobUtils.GetEuclideanDistanceXYPlane(candidatePoints[i], candidatePoints[j]) < Thresholds.HaarMaxDistanceBetweenCandidates)
                    {
                        groupedSet.Union(i, j);
                    }
                }
            }

            return groupedSet;
        }

        /// <summary>
        /// Filters the candidate points. Only the candidates that has a certain amount of possible candidates in the neighborhood area are kept.
        /// </summary>
        private static List<int> FilterCandidates(DisjointSet disjointSet)
        {
            var setRepresentatives = new List<int>();

            for (int i = 0; i < disjointSet.Count; i++)
            {
                int setRepresentative = disjointSet.Find(i);
                if (!setRepresentatives.Contains(setRepresentative) && disjointSet.SetSize(setRepresentative) > Thresholds.HaarCandidateMinCount)
                {
                    setRepresentatives.Add(setRepresentative);
                }
            }

            var highestCandidates = new List<int>();

            foreach (var representative in setRepresentatives)
            {
                highestCandidates.Add(disjointSet.HighestIndexInTree[representative]);
            }

            return highestCandidates;
        }
        
    }
}
