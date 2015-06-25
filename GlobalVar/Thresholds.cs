//
// Written by Leif Erik Bjoerkli
//


namespace InteractionDetection
{
    static class Thresholds
    {
        public const float ConnectedComponentLabelingMaxDistanceBetweenPoints = 0.03f;

        public const float HaarDifferenceBetweenInnerAndOuterRectangle = -0.15f;
        public const float HaarRectangleMaxDistance = 15.0f;
        public const int HaarRectangleMinCount = 5;

        public const float GeodesicGraphMaxDistanceBetweenPoints = 0.2f;
        public const float GeodesicGraphMaxDistanceToCandidates = 1.0f;

        public const float HandPointGroupingMaxDistance = 0.05f;
        public const int HandPointMinCount = 3;
        public const int HeadPointMinCount = 10;
        public const int TorsoPointMinCount = 10;

        public const float LastTenFramesHeadMaxDistance = 1f;
        public const float LastTenFramesTorsoMaxDistance = 0.5f;
        public const float LastTenFramesHandMaxDistance = 2f;
        public const double HeadSphereMaxError = 0.01;
    }
}
