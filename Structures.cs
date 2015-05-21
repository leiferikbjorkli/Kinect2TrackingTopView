using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Microsoft.Samples.Kinect.DepthBasics
{
    public struct Edge : IComparable<Edge>
    {
        public float weight;
        public int firstComponent, secondComponent;

        public int CompareTo(Edge other)
        {
            return this.weight.CompareTo(other.weight);
        }
    }
    public struct Component
    {
        public int rank, componentLabel, size;
    }

    public struct Rectangle
    {
        public int xStart, yStart, width, height;
    }

    public struct ThreePointRectangle
    {
        public ThreePointRectangle(Point a, Point b, Point c)
        {
            this.a = a;
            this.b = b;
            this.c = c;
        }

        public Point a, b, c;
    }
    public struct Point
    {
        public Point(int x, int y)
        {
            this.x = x;
            this.y = y;
        }
        public int x, y;
    }

    public struct IndexRectangle
    {
        public IndexRectangle(int a, int b, int c)
        {
            this.a = a;
            this.b = b;
            this.c = c;
        }
        public int a, b, c;
    }
}
