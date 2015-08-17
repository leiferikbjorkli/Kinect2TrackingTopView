//
// Copyright (c) Leif Erik Bjoerkli, Norwegian University of Science and Technology, 2015.
// Distributed under the MIT License.
// (See accompanying file LICENSE or copy at http://opensource.org/licenses/MIT)
//  
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Kinect2TrackingTopView
{
    /// <summary>
    /// Class that keeps track of the tracking history from in the ten last frames.
    /// </summary>
    static class BodiesHistory
    {
        private static Queue<List<Body>> _bodiesHistory;
        
        public static Queue<List<Body>> Get
        {
            get { return _bodiesHistory; }
        }

        public static void Initialize()
        {
            _bodiesHistory = new Queue<List<Body>>();

            for (int i = 0; i < 10; i++)
            {
                var emptyBodies = new List<Body>();
                _bodiesHistory.Enqueue(emptyBodies);
            }
        }

        public static void Update(List<Body> newBodies)
        {
            if (newBodies.Count > 0)
            {
                _bodiesHistory.Enqueue(newBodies);
            }
            if (_bodiesHistory.Count > 10)
            {
                _bodiesHistory.Dequeue();
            }
        }
    }
}
