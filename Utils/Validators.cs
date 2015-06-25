//
// Written by Leif Erik Bjoerkli
//


using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;

namespace InteractionDetection
{
    class Validators
    {
        private const float RadiusHumanHead = 0.0875f;
        //private const float RadiusHumanHead = 0.04f;

        public static bool SizeOfHead(List<int> headPointIndexes )
        {
            int count = headPointIndexes.Count;
            return count > 50 && count < 150;
        }

        public static bool IsSphericalShape(List<int> headPointIndexes,int highestPointIndex)
        {
            CameraSpacePoint highestPoint = GlobVar.MedianFilteredPointCloud[highestPointIndex];

            CameraSpacePoint headCenter = new CameraSpacePoint()
            {
                X = highestPoint.X,
                Y = highestPoint.Y,
                Z = highestPoint.Z - RadiusHumanHead
            };

            double totalError = 0.0;

            foreach (var headPointIndex in headPointIndexes)
            {
                CameraSpacePoint headPoint = GlobVar.MedianFilteredPointCloud[headPointIndex];
                double error = RadiusHumanHead*RadiusHumanHead -
                               (headPoint.Z - headCenter.Z)*(headPoint.Z - headCenter.Z) +
                               (headPoint.Y - headCenter.Y)*(headPoint.Y - headCenter.Y) +
                               (headPoint.X - headCenter.X)*(headPoint.X - headCenter.X);

                totalError += Math.Abs(error);
            }
            double meanError = totalError/headPointIndexes.Count;

            return meanError < Thresholds.HeadSphereMaxError;
        }

        // Create method to check if hypothetical volume of head would be
        // correct

    }
}
