//
// Copyright (c) Leif Erik Bjoerkli, Norwegian University of Science and Technology, 2015.
// Distributed under the MIT License.
// (See accompanying file LICENSE or copy at http://opensource.org/licenses/MIT)
//  

// Inspired by Bjarki Ágúst's implementation at
// http://www.mathblog.dk/disjoint-set-data-structure/
//

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;

namespace Kinect2TrackingTopView
{

    public class DisjointSet3D
    {
        /// <summary>
        /// Average position of points in set.
        /// </summary>
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
        private readonly int[] _parent;

        /// <summary>
        /// The rank of each element in the universe.
        /// </summary>
        private readonly int[] _rank;

        private readonly int[] _sizeOfSet;

        /// <summary>
        /// The number of disjoint sets.
        /// </summary>
        public int SetCount { get; private set; }

        public DisjointSet3D(List<CameraSpacePoint> candidates)
        {

            Count = candidates.Count();
            SetCount = Count;
            AverageX = new float[Count];
            AverageY = new float[Count];
            AverageZ = new float[Count];
            _parent = new int[Count];
            _rank = new int[Count];
            _sizeOfSet = new int[Count];

            for (int i = 0; i < Count; i++)
            {
                _parent[i] = i;
                _rank[i] = 0;
                _sizeOfSet[i] = 1;
                AverageX[i] = candidates[i].X;
                AverageY[i] = candidates[i].Y;
                AverageZ[i] = candidates[i].Z;
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
            if (_parent[i] == i)
            {
                return i;
            }
            // Recursively find the real parent of i, and then cache it for
            // later lookups.
            _parent[i] = Find(_parent[i]);
            return _parent[i];
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
            int irep = Find(i),
                jrep = Find(j),
                irank = _rank[irep],
                jrank = _rank[jrep];

            if (irep == jrep)
                return;

            SetCount--;

            //Calculate set average position
            int sizeSetI = _sizeOfSet[irep];
            int sizeSetJ = _sizeOfSet[jrep];
            int totalSizeSets = sizeSetI + sizeSetJ;

            float averageX = (AverageX[irep] * sizeSetI / totalSizeSets + AverageX[jrep] * sizeSetJ / totalSizeSets);
            float averageY = (AverageY[irep] * sizeSetI / totalSizeSets + AverageY[jrep] * sizeSetJ / totalSizeSets);
            float averageZ = (AverageZ[irep] * sizeSetI / totalSizeSets + AverageZ[jrep] * sizeSetJ / totalSizeSets);

            if (irank < jrank)
            {
                _parent[irep] = jrep;
                _sizeOfSet[jrep] += sizeSetI;

                AverageX[jrep] = averageX;
                AverageY[jrep] = averageY;
                AverageZ[jrep] = averageZ;
            } 
            else if (jrank < irank)
            {
                _parent[jrep] = irep;
                _sizeOfSet[irep] += sizeSetJ;

                AverageX[irep] = averageX;
                AverageY[irep] = averageY;
                AverageZ[irep] = averageZ;

            } 
            else
            {
                _parent[irep] = jrep;
                _sizeOfSet[jrep] += sizeSetI;

                _rank[irep]++;

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
            return _sizeOfSet[Find(i)];
        }
    }
}
