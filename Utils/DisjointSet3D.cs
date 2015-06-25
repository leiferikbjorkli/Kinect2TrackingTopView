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
    public class DisjointSet3D
    {


        public float[] AverageX { get; private set; }
        public float[] AverageY { get; private set; }
        public float[] AverageZ { get; private set; }

        /// <summary>
        /// The number of elements in the universe.
        /// </summary>
        public int Count { get; private set; }

        /// <summary>
        /// The parent of each element in the universe. Also label
        /// </summary>s
        private int[] Parent;

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
        public DisjointSet3D(List<CameraSpacePoint> candidates)
        {

            this.Count = candidates.Count();
            this.SetCount = this.Count;
            this.AverageX = new float[this.Count];
            this.AverageY = new float[this.Count];
            this.AverageZ = new float[this.Count];
            this.Parent = new int[this.Count];
            this.Rank = new int[this.Count];
            this.SizeOfSet = new int[this.Count];

            for (int i = 0; i < this.Count; i++)
            {
                this.Parent[i] = i;
                this.Rank[i] = 0;
                this.SizeOfSet[i] = 1;
                this.AverageX[i] = candidates[i].X;
                this.AverageY[i] = candidates[i].Y;
                this.AverageZ[i] = candidates[i].Z;
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
            int totalSizeSets = sizeSetI + sizeSetJ;

            float averageX = (AverageX[irep] * sizeSetI / totalSizeSets + AverageX[jrep] * sizeSetJ / totalSizeSets);
            float averageY = (AverageY[irep] * sizeSetI / totalSizeSets + AverageY[jrep] * sizeSetJ / totalSizeSets);
            float averageZ = (AverageZ[irep] * sizeSetI / totalSizeSets + AverageZ[jrep] * sizeSetJ / totalSizeSets);

            // If i's rank is less than j's rank
            if (irank < jrank)
            {

                // Then move i under j
                this.Parent[irep] = jrep;
                this.SizeOfSet[jrep] += sizeSetI;

                AverageX[jrep] = averageX;
                AverageY[jrep] = averageY;
                AverageZ[jrep] = averageZ;


            } // Else if j's rank is less than i's rank
            else if (jrank < irank)
            {

                // Then move j under i
                this.Parent[jrep] = irep;
                this.SizeOfSet[irep] += sizeSetJ;

                AverageX[irep] = averageX;
                AverageY[irep] = averageY;
                AverageZ[irep] = averageZ;

            } // Else if their ranks are the same
            else
            {

                // Then move i under j (doesn't matter which one goes where)
                this.Parent[irep] = jrep;
                this.SizeOfSet[jrep] += sizeSetI;

                // And increment the the result tree's rank by 1
                this.Rank[irep]++;

                AverageX[jrep] = averageX;
                AverageY[jrep] = averageY;
                AverageZ[jrep] = averageZ;
            }
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
