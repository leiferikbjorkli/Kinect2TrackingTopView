using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace InteractionDetection
{
    class Candidate
    {
        private List<Point> members = new List<Point>();
        private float maxInternalPointDistance;

        public Candidate(Point member)
        {
            Members.Add(member);
            maxInternalPointDistance = 0;
        }

        public List<Point> Members
        {
            get { return members; }
        }

        public void AddPoint(Point newMember,float distanceBetweenPoint)
        {
            members.Add(newMember);
            if (distanceBetweenPoint>maxInternalPointDistance)
            {
                maxInternalPointDistance = distanceBetweenPoint;
            }

        }
    }
}
