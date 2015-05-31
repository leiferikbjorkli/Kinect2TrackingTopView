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

        public Head(int label)
        {
            HeadPixels = new List<int>();
            Label = label;
        }

        public void AddPixel(int newPixel)
        {
            HeadPixels.Add(newPixel);
        }

        public bool ContainsPixel(int pixel)
        {
            return HeadPixels.Contains(pixel);
        }


    }
}
