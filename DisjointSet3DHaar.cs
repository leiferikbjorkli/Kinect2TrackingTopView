//
// Written by Leif Erik Bjoerkli
//


using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;

namespace InteractionDetection
{

    /// <summary>
    /// A Union-Find/Disjoint-Set data structure.
    /// </summary>
    public class DisjointSet3DHaar
    {
        public float[] HighestX { get; private set; }
        public float[] HighestY { get; private set; }
        public float[] HighestZ { get; private set; }

        /// <summary>
        /// The number of elements in the universe.
        /// </summary>
        public int Count { get; private set; }

        /// <summary>
        /// The parent of each element in the universe. Also label
        /// </summary>s
        public int[] Parent;

        /// <summary>
        /// The rank of each element in the universe.
        /// </summary>
        private int[] Rank;

        /// <summary>
        /// The size of each set.
        /// </summary>
        private int[] SizeOfSet;

        /// <summary>
        /// The number of disjoint sets.
        /// </summary>
        public int SetCount { get; private set; }

        /// <summary>
        /// Initializes a new Disjoint-Set data structure, with the specified amount of elements in the universe.
        /// </summary>
        /// <param name='count'>
        /// The number of elements in the universe.
        /// </param>
        public DisjointSet3DHaar(List<int> candidates)
        {

            this.Count = candidates.Count();
            this.SetCount = this.Count;
            this.HighestX = new float[this.Count];
            this.HighestY = new float[this.Count];
            this.HighestZ = new float[this.Count];
            this.Parent = new int[this.Count];
            this.Rank = new int[this.Count];
            this.SizeOfSet = new int[this.Count];

            for (int i = 0; i < this.Count; i++)
            {
                this.Parent[i] = i;
                this.Rank[i] = 0;
                this.SizeOfSet[i] = 1;
                this.HighestX[i] = GlobVar.SubtractedFilteredPointCloud[candidates[i]].X;
                this.HighestY[i] = GlobVar.SubtractedFilteredPointCloud[candidates[i]].Y;
                this.HighestZ[i] = GlobVar.SubtractedFilteredPointCloud[candidates[i]].Z;
            }
        }

        /// <summary>
        /// Find the parent of the specified element.
        /// </summary>
        /// <param name='i'>
        /// The specified element.
        /// </param>
        /// <remarks>
        /// All elements with the same parent are in the same set.
        /// </remarks>
        public int Find(int i)
        {
            if (this.Parent[i] == i)
            {
                return i;
            }
            else
            {
                // Recursively find the real parent of i, and then cache it for later lookups.
                this.Parent[i] = this.Find(this.Parent[i]);
                return this.Parent[i];
            }
        }

        /// <summary>
        /// Unite the sets that the specified elements belong to.
        /// </summary>
        /// <param name='i'>
        /// The first element.
        /// </param>
        /// <param name='j'>
        /// The second element.
        /// </param>
        public void Union(int i, int j)
        {

            // Find the representatives (or the root nodes) for the set that includes i
            int irep = this.Find(i),
                // And do the same for the set that includes j
                jrep = this.Find(j),
                // Get the rank of i's tree
                irank = this.Rank[irep],
                // Get the rank of j's tree
                jrank = this.Rank[jrep];

            // Elements are in the same set, no need to unite anything.
            if (irep == jrep)
                return;

            this.SetCount--;

            //Calculate set average position
            int sizeSetI = this.SizeOfSet[irep];
            int sizeSetJ = this.SizeOfSet[jrep];

            float highestX;
            float highestY;
            float highestZ;

            // If i's rank is less than j's rank
            if (HighestZ[irep] > HighestZ[jrep])
            {
                // Then move i under j
                this.Parent[irep] = jrep;
                this.SizeOfSet[jrep] += sizeSetI;

                highestX = HighestX[jrep];
                highestY = HighestY[jrep];
                highestZ = HighestZ[jrep];

            } // Else if j's rank is less than i's rank
            else if (HighestZ[irep] < HighestZ[jrep])
            {
                // Then move j under i
                this.Parent[jrep] = irep;
                this.SizeOfSet[irep] += sizeSetJ;

                highestX = HighestX[irep];
                highestY = HighestY[irep];
                highestZ = HighestZ[irep];
            } // Else if their ranks are the same
            else
            {
                // Then move i under j (doesn't matter which one goes where)
                this.Parent[irep] = jrep;
                this.SizeOfSet[jrep] += sizeSetI;

                highestX = HighestX[jrep];
                highestY = HighestY[jrep];
                highestZ = HighestZ[jrep];

                // And increment the the result tree's rank by 1
                this.Rank[irep]++;
            }

            HighestX[jrep] = highestX;
            HighestY[jrep] = highestY;
            HighestZ[jrep] = highestZ;

            HighestX[irep] = highestX;
            HighestY[irep] = highestY;
            HighestZ[irep] = highestZ;

            HighestX[i] = highestX;
            HighestY[i] = highestY;
            HighestZ[i] = highestZ;

            HighestX[j] = highestX;
            HighestY[j] = highestY;
            HighestZ[j] = highestZ;

        }

        /// <summary>
        /// Return the element count of the set that the specified elements belong to.
        /// </summary>
        /// <param name='i'>
        /// The element.
        /// </param>
        public int SetSize(int i)
        {
            return this.SizeOfSet[this.Find(i)];
        }
    }
}
