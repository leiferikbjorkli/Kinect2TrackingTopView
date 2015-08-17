//
// Copyright (c) Leif Erik Bjoerkli, Norwegian University of Science and Technology, 2015.
// Distributed under the MIT License.
// (See accompanying file LICENSE or copy at http://opensource.org/licenses/MIT)
//  
namespace Kinect2TrackingTopView
{
    public struct Point
    {
        public Point(int x, int y)
        {
            X = x;
            Y = y;
        }
        public int X, Y;
    }
    public struct IndexRectangle
    {
        public IndexRectangle(int a, int b, int c)
        {
            A = a;
            B = b;
            C = c;
        }

        public int A, B, C;
    }

   
}
