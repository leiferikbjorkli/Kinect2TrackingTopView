using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Microsoft.Samples.Kinect.DepthBasics
{

    class SegmentationSet
    {
        private int numberOfComponents;

        public int NumberOfComponents
        {
            get { return numberOfComponents; }
            set { numberOfComponents = value; }
        }
        private Component[] components;

        public Component[] Components
        {
            get { return components; }
            set { components = value; }
        }

        public unsafe SegmentationSet(int numberOfSegments)
        {
            this.numberOfComponents = numberOfSegments;
            this.components = new Component[numberOfComponents];
            fixed (Component* pointerComponents = components)
            {
                for (int i = 0; i < numberOfSegments; i++)
                {
                    pointerComponents[i].rank = 0;
                    pointerComponents[i].size = 1;
                    pointerComponents[i].componentLabel = i;
                }
            
            }

        }

        public int size(int componentIndex) { return this.components[componentIndex].size;}

        public unsafe int find(int componentIndex)
        {
            fixed (Component* pointerComponents = components)
            {
                int y = componentIndex;
                while (y != pointerComponents[y].componentLabel)
                {
                    y = pointerComponents[y].componentLabel;
                }
                pointerComponents[componentIndex].componentLabel = y;
                return y;
            }
        }

        public unsafe void join(int firstComponentIndex, int secondComponentIndex)
        {
            fixed (Component* PComponents = components)
            {
                if (PComponents[firstComponentIndex].rank > PComponents[secondComponentIndex].rank)
                {
                    PComponents[secondComponentIndex].componentLabel = firstComponentIndex;
                    PComponents[firstComponentIndex].size += PComponents[secondComponentIndex].size;
                } else {
                    PComponents[firstComponentIndex].componentLabel = secondComponentIndex;
                    PComponents[secondComponentIndex].size += PComponents[firstComponentIndex].size;
                    if (PComponents[firstComponentIndex].rank == PComponents[secondComponentIndex].rank)
                    {
                        PComponents[secondComponentIndex].rank++;
                    }
                }
                numberOfComponents--;            
            }
        }
    }
}
