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
    class Body
    {
        public Hand[] Hands { get; set; }
        public Head Head { get; private set; }
        public Torso Torso { get; set; }
        public int Id { get; set; }
        public TimeSpan TimeStamp { get; set; }
        public double EnergyLevel { get; set; }

        public CameraSpacePoint[] HeadPoints { get; set; }
        //public CameraSpacePoint[] HandPoints { get; set; }
        //public CameraSpacePoint[] HandPoints { get; set; }
        //public CameraSpacePoint[] HandPoints { get; set; }


        public Body(int id, TimeSpan timeStamp)
        {
            Id = id;
            Hands = new Hand[2];
            TimeStamp = timeStamp;
        }

        public int AddHead(Head head)
        {

            Head = head;
            return 1;
        }

        public void AddHands(Hand[] hands)
        {
            Hands = hands;
        }

        public void AddTorso(Torso torso)
        {
            Torso = torso;
        }


    }
}
