//
// Copyright (c) Leif Erik Bjoerkli, Norwegian University of Science and Technology, 2015.
// Distributed under the MIT License.
// (See accompanying file LICENSE or copy at http://opensource.org/licenses/MIT)
//

using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Kinect;

namespace Kinect2TrackingTopView
{
    /// <summary>
    /// Class that contain validators used to validate candidate heads blobs.
    /// </summary>
    static class Validators
    {
        /// <summary>
        /// Calculates the accumulated difference in depths from the current frame and the previous for certain indexes.
        /// </summary>
        /// <remarks>
        /// NOT USED
        /// </remarks>
        public static bool EvaluateMovementInFrame(List<int> headPointIndexes, float minDifferencePreviousFrame)
        {
            float differenceFromPreviousFrame = 0;

            for (int i = 0; i < headPointIndexes.Count; i++)
            {
                differenceFromPreviousFrame +=
                    Math.Abs(GlobVar.PreviousFrame[headPointIndexes[i]].Z - GlobVar.SubtractedFilteredPointCloud[headPointIndexes[i]].Z);
            }

            return differenceFromPreviousFrame >= minDifferencePreviousFrame;
        }

        /// <summary>
        /// Evaluates the size of the candidate head region against thresholds for maximum and minimum size.
        /// </summary>
        public static bool EvaluateSizeOfHeadRegion(List<int> headPointIndexes )
        {
            int count = headPointIndexes.Count;
            return count > Thresholds.ValidatorsHeadPointMinCount && count < Thresholds.ValidatorsHeadPointMaxCount;
        }

        /// <summary>
        /// Evaluates the difference between input head region and a average head spherical shape.
        /// </summary>
        /// /// <remarks>
        /// NOT USED
        /// </remarks>
        public static bool IsSphericalShape(List<int> headPointIndexes, int highestPointIndex, double headSphereMaxError)
        {
            const float radiusHumanHead = GlobVar.RadiusHumanHead;
            const float maxDepth = GlobVar.MaxSensingDepth;

            CameraSpacePoint highestPoint = GlobVar.SubtractedFilteredPointCloud[highestPointIndex];

            var headCenter = new CameraSpacePoint()
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

            return meanError < headSphereMaxError;
        }

    }
}
