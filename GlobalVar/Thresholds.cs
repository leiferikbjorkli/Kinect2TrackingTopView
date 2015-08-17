//
// Written by Leif Erik Bjoerkli
//


namespace InteractionDetection
{
    static class Thresholds
    {
        public const float ClassificationLabelingMaxDistanceBetweenPoints = 0.05f;
        //public const int ClassificationLabelingMaxPoints = 200; Alex

        //public const int ClassificationLabelingMaxPoints = 150; Girl
        public const int ClassificationLabelingMaxPoints = 110;
        public const int ClassificationSearchRangeHighestPoint = 100;
        public const int ClassificationSearchDepthHighestPoint = 20;

        
        //public const float HaarDifferenceBetweenInnerAndOuterRectangle = 0.7f;
        //Alex

        public const float HaarDifferenceBetweenInnerAndOuterRectangle = 0.5f;

        //public const float HaarRectangleMaxDistance = 0.05f;
        public const float HaarRectangleMaxDistance = 0.1f;
        public const int HaarRectangleMinCount = 15;

        public const float GeodesicGraphMaxDistanceBetweenPoints = 0.2f;
        public const float GeodesicGraphMaxDistanceToCandidates = 1f;

        public const int ValidatorsHeadPointMinCount = 100;
        //public const int ValidatorsHeadPointMaxCount = 210; Alex
        public const int ValidatorsHeadPointMaxCount = 120;
        public const float ValidatorsMinDifferencePreviousFrame = 0f;
        public const float ValidatorsHighestPointGroupingDistance = 0.5f;
        //public const double ValidatorsHeadSphereMaxError = 0.04;
        public const double ValidatorsHeadSphereMaxError = 1f;

        public const float LastFramesHeadMaxDistance = 2f;
        public const float LastFramesTorsoMaxDistance = 0.2f;
        public const float LastFramesHandMaxDistance = 0.3f;

        public const float HandPointGroupingMaxDistance = 0.05f;
        public const int HandPointMinCount = 3;
        public const int TorsoPointMinCount = 10;

        public const float DetectionHeightHeadMin = 0.2f;

        public const float TemporalMedianMapPadding = 0.02f;
    }
}
