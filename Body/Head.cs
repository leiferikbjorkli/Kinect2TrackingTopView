using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;

namespace InteractionDetection
{
    class Head
    {
        public List<int> HeadPixels { get; private set; }
        public int Label { get; private set; }
        public int CenterPoint;

        public void AddHeadPixels(List<int> headPixels )
        {
            HeadPixels = headPixels;
            CenterPoint = BodyUtils.CalculateCenterPointHeadPoints(headPixels);
        }
    }
}
