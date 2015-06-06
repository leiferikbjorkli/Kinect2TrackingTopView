using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace InteractionDetection
{
    static class Thresholds
    {
        public const float Labeling = 0.03f;
        public const float SlidingWindow = 0.05f;
        public const float Grouping = 20.0f;
        public const float FilterCandidateDistance = 50f;
        public const float HaarRectangleMaxDistance = 15.0f;
        public const float GeodesicGraph = 0.1f;
        public const double HandGroupingAngle = 20;
        public const int HaarRectangleCount = 10;
        public const float MaxDistanceLastFrameHead = 5f;
    }
}
