//
// Copyright (c) Leif Erik Bjoerkli, Norwegian University of Science and Technology, 2015.
// Distributed under the MIT License.
// (See accompanying file LICENSE or copy at http://opensource.org/licenses/MIT)
//  

namespace Kinect2TrackingTopView
{
    static class Thresholds
    {
        public const float TemporalMedianMapPadding = 0.02f;

        public const float ClassificationLabelingMaxHeightBetweenPoints = 0.05f;
        public const int ClassificationLabelingMaxPoints = 150;
        public const int ClassificationHighestPointSearchDepth = 20;
        public const float ClassificationHighestPointGroupingDistance = 0.5f;

        public const float HaarDifferenceBetweenInnerAndOuterRectangle = 0.5f;
        public const float HaarMaxDistanceBetweenCandidates = 0.1f;
        public const int HaarCandidateMinCount = 15;
        public const float HaarDetectionHeightHeadMin = 0.2f;

        public const float GeodesicGraphMaxDistanceBetweenPoints = 0.2f;
        public const float GeodesicGraphMaxDistanceToCandidates = 1f;

        public const int ValidatorsHeadPointMaxCount = 170;
        public const int ValidatorsHeadPointMinCount = 100;

        public const float LastFramesHeadMaxDistance = 2f;
        public const float LastFramesTorsoMaxDistance = 0.2f;
        public const float LastFramesHandMaxDistance = 0.3f;

        public const float HandPointGroupingMaxDistance = 0.05f;
        public const int HandPointMinCount = 3;
        public const int TorsoPointMinCount = 10;

    }
}
