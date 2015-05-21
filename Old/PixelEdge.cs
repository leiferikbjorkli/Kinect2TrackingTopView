using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;

namespace Microsoft.Samples.Kinect.DepthBasics
{
    class PixelEdge:IComparable<PixelEdge>
    {
        private PixelVertex[] vertices = new PixelVertex[2];
        private float weight;



        public PixelEdge(PixelVertex[] vertices, float weight)
        {
            this.vertices = vertices;
            this.weight = weight;
        }

        internal PixelVertex[] Vertices
        {
            get { return vertices; }
            set { vertices = value; }
        }

        public int CompareTo(PixelEdge otherEdge)
        {
            if (otherEdge == null) return 1;

            return this.weight.CompareTo(otherEdge.weight);
        }

        public float Weight
        {
            get { return this.weight; }
            set { this.weight = value; }
        }

    }

}
