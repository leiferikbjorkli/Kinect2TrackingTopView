//
// Written by Leif Erik Bjoerkli
//


using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Kinect;

namespace InteractionDetection
{
    class Validators
    {

        // If shape is circular

        public static List<int> GroupCandidateHighestPoints(List<int> highestPointIndexes, float groupingMaxDistance)
        {
            List<int> groupedPoints = new List<int>();

            Dictionary<int,float> pointDepth = new Dictionary<int, float>();
            for (int i = 0; i < highestPointIndexes.Count; i++)
            {
                if (!pointDepth.ContainsKey(highestPointIndexes[i]))
                {
                    pointDepth.Add(highestPointIndexes[i],GlobVar.SubtractedFilteredPointCloud[highestPointIndexes[i]].Z);
                }
            }

            var sortedPointDepth = pointDepth.OrderBy(kvp => kvp.Value);

            foreach (var sortedPoint in sortedPointDepth)
            {
                int currentIndex = sortedPoint.Key;
                bool add = true;
                
                foreach (var groupedPoint in groupedPoints)
                {
                    if (GlobUtils.GetEuclideanDistanceXYPlane(groupedPoint,currentIndex) < groupingMaxDistance)
                    {
                        add = false;
                    }
                }
                if (add)
                {
                    groupedPoints.Add(sortedPoint.Key);
                }
            }

            return groupedPoints;
        }

        public static bool MovementInFrame(List<int> headPointIndexes)
        {
            float differenceFromPreviousFrame = 0;

            for (int i = 0; i < headPointIndexes.Count; i++)
            {
                //Console.WriteLine(GlobVar.PreviousFrame[i].Z);
                //Console.WriteLine(GlobVar.SubtractedFilteredPointCloud[i].Z);

                differenceFromPreviousFrame +=
                    Math.Abs(GlobVar.PreviousFrame[headPointIndexes[i]].Z - GlobVar.SubtractedFilteredPointCloud[headPointIndexes[i]].Z);

            }

            return differenceFromPreviousFrame >= Thresholds.ValidatorsMinDifferencePreviousFrame;
        }

        // Evaluate size compared to height
        public static bool SizeOfHead(List<int> headPointIndexes )
        {
            int count = headPointIndexes.Count;
            //Console.WriteLine(count);
            return count > Thresholds.ValidatorsHeadPointMinCount && count < Thresholds.ValidatorsHeadPointMaxCount;
        }

        public static bool IsSphericalShape(List<int> headPointIndexes,int highestPointIndex)
        {
            var radiusHumanHead = GlobVar.RadiusHumanHead;
            var maxDepth = GlobVar.MaxDepthMeter;

            CameraSpacePoint highestPoint = GlobVar.SubtractedFilteredPointCloud[highestPointIndex];

            CameraSpacePoint headCenter = new CameraSpacePoint()
            {
                X = highestPoint.X,
                Y = highestPoint.Y,
                Z = highestPoint.Z - radiusHumanHead
            };

            double totalError = 0.0;
            int validPixelCount = 0;

            foreach (var headPointIndex in headPointIndexes)
            {
                CameraSpacePoint headPoint = GlobVar.SubtractedFilteredPointCloud[headPointIndex];

                if (headPoint.Z != maxDepth && !float.IsInfinity(headPoint.X) && !float.IsInfinity(headPoint.Y))
                {
                    double error = radiusHumanHead*radiusHumanHead -
                                   (headPoint.Z - headCenter.Z)*(headPoint.Z - headCenter.Z) +
                                   (headPoint.Y - headCenter.Y)*(headPoint.Y - headCenter.Y) +
                                   (headPoint.X - headCenter.X)*(headPoint.X - headCenter.X);

                    totalError += Math.Abs(error);
                    validPixelCount++;
                }
            }
            double meanError = totalError / validPixelCount;

            //Console.WriteLine(meanError);

            return meanError < Thresholds.ValidatorsHeadSphereMaxError;
        }

        // Create method to check if hypothetical volume of head would be
        // correct

    }
}
