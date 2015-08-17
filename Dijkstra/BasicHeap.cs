//
// Copyright (c) Leif Erik Bjoerkli, Norwegian University of Science and Technology, 2015.
// Distributed under the MIT License.
// (See accompanying file LICENSE or copy at http://opensource.org/licenses/MIT)
//  

//
// Inspired by Tolga Birdals implementation in
// http://www.codeproject.com/Articles/24816/A-Fast-Priority-Queue-Implementation-of-the-Dijkst
//

using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;

namespace Kinect2TrackingTopView
{
    public struct HeapNode
    {
        public HeapNode(int i, float w)
        {
            Index = i;
            Weight = w;

        }
        public int Index;
        public float Weight;
    }

    class BasicHeap 
    {
        private readonly List<HeapNode> _innerList = new List<HeapNode>();

        public int Count
        {
            get
            {
                return _innerList.Count;
            }
        }

        public void Push(int index, float weight)
        {
            int p = _innerList.Count, p2;
            _innerList.Add(new HeapNode(index,weight));
            do
            {
                if (p == 0)
                    break;

                p2 = (p - 1) >> 1;
                if (_innerList[p].Weight<_innerList[p2].Weight)
                {
                    HeapNode h = _innerList[p];
                    _innerList[p] = _innerList[p2];
                    _innerList[p2] = h;

                    p = p2;
                }
                else
                    break;
            } while (true);
        }

        public int Pop()
        {
            HeapNode result = _innerList[0];
            int p = 0, p1, p2, pn;
            _innerList[0] = _innerList[_innerList.Count - 1];
            _innerList.RemoveAt(_innerList.Count - 1);
            do
            {
                pn = p;
                p1 = (p << 1) + 1;
                p2 = (p << 1)  + 2;
                if (_innerList.Count > p1 && _innerList[p].Weight > _innerList[p1].Weight) 
                    p = p1;
                if (_innerList.Count > p2 && _innerList[p].Weight > _innerList[p2].Weight) 
                    p = p2;

                if (p == pn)
                    break;
                
                HeapNode h = _innerList[p];
                _innerList[p] = _innerList[pn];
                _innerList[pn] = h;

            } while (true);
            return result.Index;
        }
    }
}
