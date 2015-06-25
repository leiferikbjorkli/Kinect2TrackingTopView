//
// Written by Leif Erik Bjoerkli
//


using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Windows.Media.Media3D;

namespace InteractionDetection
{
    class EnergyUtils
    {

        private const double Gravity = 9.81f;
        private const double BodyMass = 62f;
        private const double HeadMass = BodyMass * 0.23f;
        private const double TorsoMass = BodyMass * 0.58f;
        private const double ForeArmMass = BodyMass * 0.018f;
        private const double UpperArmMass = BodyMass * 0.035f;

        // Head moves up with body kinetic and potential energy

        public static void UpdateEnergy(Body body)
        {

            List<Body> lastBodies = BodyUtils.GetBodiesFromHistory(body.Id);

            if (lastBodies.Count < 2)
            {
                body.EnergyLevel = 0;
                return;
            }

            List<Body> lastTwoBodies = new List<Body>();
            lastTwoBodies.Add(lastBodies[lastBodies.Count - 1]);
            lastTwoBodies.Add(lastBodies[lastBodies.Count - 2]);


            double totalPotential = 0;
            double totalKinetic = 0;

            //Vector3D headDisplacement = CalculateDisplacementHead(lastBodies);
            //totalPotential += CalculatePotentialEnergyChange(headDisplacement,
            //HeadMass); totalKinetic +=
            //CalculateKineticEnergyChange(headDisplacement, HeadMass);

            //Vector3D torsoDisplacement = CalculateDisplacementTorso(lastBodies);
            //totalPotential += CalculatePotentialEnergyChange(torsoDisplacement, TorsoMass);
            //totalKinetic += CalculateKineticEnergyChange(torsoDisplacement, TorsoMass);

            Vector3D[] handDisplacement = CalculateDisplacementHands(lastBodies);
            totalPotential += CalculatePotentialEnergyChange(handDisplacement[0], ForeArmMass);
            totalKinetic += CalculateKineticEnergyChange(handDisplacement[0], ForeArmMass);

            totalPotential += CalculatePotentialEnergyChange(handDisplacement[1], ForeArmMass);
            totalKinetic += CalculateKineticEnergyChange(handDisplacement[1], ForeArmMass);

            body.EnergyLevel = totalKinetic + totalPotential;
        }

        private static Vector3D CalculateDisplacementHead(List<Body> bodies)
        {
            float displacementX = 0;
            float displacementY = 0;
            float displacementZ = 0;

            for (int i = 1; i < bodies.Count; i++)
            {
                displacementX += Math.Abs(bodies[i].Head.CenterPoint.X - bodies[i - 1].Head.CenterPoint.X);
                displacementY += Math.Abs(bodies[i].Head.CenterPoint.Y - bodies[i - 1].Head.CenterPoint.Y);
                displacementZ += Math.Abs(bodies[i].Head.CenterPoint.Z - bodies[i - 1].Head.CenterPoint.Z);
            }

            return new Vector3D(displacementX, displacementY, displacementZ);
        }

        private static Vector3D CalculateDisplacementTorso(List<Body> bodies)
        {
            float displacementX = 0;
            float displacementY = 0;
            float displacementZ = 0;

            for (int i = 1; i < bodies.Count; i++)
            {
                if (bodies[i].Torso != null && bodies[i-1].Torso != null)
                {
                    displacementX += Math.Abs(bodies[i].Torso.CenterPoint.X - bodies[i - 1].Torso.CenterPoint.X);
                    displacementY += Math.Abs(bodies[i].Torso.CenterPoint.Y - bodies[i - 1].Torso.CenterPoint.Y);
                    displacementZ += Math.Abs(bodies[i].Torso.CenterPoint.Z - bodies[i - 1].Torso.CenterPoint.Z); 
                }
            }

            return new Vector3D(displacementX, displacementY, displacementZ);
        }

        private static Vector3D[] CalculateDisplacementHands(List<Body> bodies)
        {
            float displacementFirstHandX = 0;
            float displacementFirstHandY = 0;
            float displacementFirstHandZ = 0;

            float displacementSecondHandX = 0;
            float displacementSecondHandY = 0;
            float displacementSecondHandZ = 0;

            for (int i = 1; i < bodies.Count; i++)
            {
                if (bodies[i].Hands[0] != null && bodies[i-1].Hands[0] != null)
                {
                    displacementFirstHandX = Math.Abs(bodies[i].Hands[0].CenterPoint.X - bodies[i - 1].Hands[0].CenterPoint.X);
                    displacementFirstHandY = Math.Abs(bodies[i].Hands[0].CenterPoint.Y - bodies[i - 1].Hands[0].CenterPoint.Y);
                    displacementFirstHandZ = Math.Abs(bodies[i].Hands[0].CenterPoint.Z - bodies[i - 1].Hands[0].CenterPoint.Z); 
                }
                if (bodies[i].Hands[1] != null && bodies[i - 1].Hands[1] != null)
                {
                    displacementSecondHandX = Math.Abs(bodies[i].Hands[1].CenterPoint.X - bodies[i - 1].Hands[1].CenterPoint.X);
                    displacementSecondHandY = Math.Abs(bodies[i].Hands[1].CenterPoint.Y - bodies[i - 1].Hands[1].CenterPoint.Y);
                    displacementSecondHandZ = Math.Abs(bodies[i].Hands[1].CenterPoint.Z - bodies[i - 1].Hands[1].CenterPoint.Z);
                }
            }
            var displacementFirstHand = new Vector3D(displacementFirstHandX,displacementFirstHandY,displacementFirstHandZ);
            var displacementSecondHand  = new Vector3D(displacementSecondHandX,displacementSecondHandY,displacementSecondHandZ);

            return new Vector3D[]{displacementFirstHand,displacementSecondHand};
        }

        private static double CalculatePotentialEnergyChange(Vector3D displacement, double mass)
        {
            return mass*Gravity*displacement.Z;
        }

        private static double CalculateKineticEnergyChange(Vector3D displacement, double mass)
        {
            double deltaTime = BodyUtils.GetTimeSpanLastTenFrames();
            if (deltaTime == 0)
            {
                return 0;
            }

            double meanVelocityX = displacement.X/deltaTime;
            double meanVelocityY = displacement.Y/deltaTime;

            double kineticEnergyX = 0.5*mass*meanVelocityX*meanVelocityX;
            double kineticEnergyY = 0.5*mass*meanVelocityY*meanVelocityY;

            return kineticEnergyX + kineticEnergyY;
        }
    }
}
