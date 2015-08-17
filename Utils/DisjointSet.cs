//
// Copyright (c) Leif Erik Bjoerkli, Norwegian University of Science and Technology, 2015.
// Distributed under the MIT License.
// (See accompanying file LICENSE or copy at http://opensource.org/licenses/MIT)
//  

//
// Inspired by Bjarki Ágúst's implementation at
// http://www.mathblog.dk/disjoint-set-data-structure/
//

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Kinect2TrackingTopView {

    public class DisjointSet {

        public int[] HighestIndexInTree { get; private set; }

        /// <summary>
        /// The number of elements in the universe.
        /// </summary>
        public int Count { get; private set; }

        /// <summary>
        /// The parent of each element in the universe.
        /// </summary>
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


        public DisjointSet(List<int> candidates) {

            Count = candidates.Count();
            SetCount = Count;
            _parent = new int[Count];
            _rank = new int[Count];
            _sizeOfSet = new int[Count];
            HighestIndexInTree = new int[Count];

            for (int i = 0; i < Count; i++) {
                _parent[i] = i;
                _rank[i] = 0;
                _sizeOfSet[i] = 1;
                HighestIndexInTree[i] = candidates[i];
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
        public int Find(int i) {
            if (_parent[i] == i) {
                return i;
            }
            // Recursively find the real parent of i, and then cache it for later lookups.
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
        public void Union(int i, int j) {
         
            // Find the representatives (or the root nodes)
            int irep = Find(i),
                jrep = Find(j),
                irank = _rank[irep],
                jrank = _rank[jrep];

            if (irep == jrep)
                return;

            SetCount--;

            // Calculate set average position
            int sizeSetI = _sizeOfSet[irep];
            int sizeSetJ = _sizeOfSet[jrep];

            int highestIndex;

            if (GlobVar.SubtractedFilteredPointCloud[HighestIndexInTree[irep]].Z < GlobVar.SubtractedFilteredPointCloud[HighestIndexInTree[jrep]].Z)
            {
                highestIndex = HighestIndexInTree[irep];
            }
            else
            {
                highestIndex = HighestIndexInTree[jrep];
            }

            if (irank < jrank) {
         
                _parent[irep] = jrep;
                _sizeOfSet[jrep] += sizeSetI;
                HighestIndexInTree[jrep] = highestIndex;

            }
            else if (jrank < irank) {
         
                _parent[jrep] = irep;
                _sizeOfSet[irep] += sizeSetJ;
                HighestIndexInTree[irep] = highestIndex;
                
            }
            else {
         
                _parent[irep] = jrep;
                _sizeOfSet[jrep] += sizeSetI;
                HighestIndexInTree[jrep] = highestIndex;
         
                _rank[irep]++;
                
            }
        }

        /// <summary>
        /// Return the element count of the set that the specified elements belong to.
        /// </summary>
        /// <param name='i'>
        /// The element.
        /// </param>
        public int SetSize(int i) {
            return _sizeOfSet[Find(i)];
        }
    }
}
