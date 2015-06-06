using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace InteractionDetection
{
    class Body
    {
        public Head Head { get; private set; }
        public LeftHand LeftHand { get; private set; }
        public RightHand RightHand { get; private set; }
        public LeftShoulder LeftShoulder { get; private set; }
        public RightShoulder RightShoulder { get; private set; }

        public Body(Head head)
        {
            Head = head;
        }
        public void AddLeftHand(LeftHand leftHand)
        {
            LeftHand = leftHand;
        }
        public void AddRightHand(RightHand rightHand)
        {
            RightHand = rightHand;
        }

        public void AddRightShoulder(RightShoulder rightShoulder)
        {
            RightShoulder = rightShoulder;
        }

        public void AddLeftShoulder(LeftShoulder leftShoulder)
        {
            LeftShoulder = leftShoulder;
        }

    }
}
