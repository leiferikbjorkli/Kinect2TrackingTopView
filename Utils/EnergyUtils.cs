//
// Copyright (c) Leif Erik Bjoerkli, Norwegian University of Science and Technology, 2015.
// Distributed under the MIT License.
// (See accompanying file LICENSE or copy at http://opensource.org/licenses/MIT)
//  
using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media.Media3D;

namespace Kinect2TrackingTopView
{
    /// <summary>
    /// Class that calculates the mechanical energy use for each frame.
    /// </summary>
    static class EnergyUtils
    {
        private const double Gravity = 9.81f;
        private const double BodyMass = 50f;
        private const double HeadMass = BodyMass * 0.23f;
        private const double TorsoMass = BodyMass * 0.58f;
        private const double ForeArmMass = BodyMass * 0.018f;
        private const double UpperArmMass = BodyMass * 0.035f;

        /// <summary>
        /// Attributes the body object with the calculated mechanical energy-use since last frame.
        /// </summary>
        public static void CalculateBodyMechanicalEnergy(Body body)
        {
            bool handTracking = false;

            List<Body> lastBodies = BodyUtils.GetBodiesWithIdFromHistory(body.Id);

            if (lastBodies.Count < 1)
            {
                body.EnergyLevel = 0;
                return;
            }

            Body[] lastTwoBodies = new Body[2];
            lastTwoBodies[0] = lastBodies[lastBodies.Count - 1];
            lastTwoBodies[1] = body;

            double totalPotential = 0;
            double totalKinetic = 0;

            Vector3D headDisplacement = CalculateDisplacementHead(lastTwoBodies);
            totalPotential += CalculatePotentialEnergyChange(headDisplacement, HeadMass);
            totalKinetic += CalculateKineticEnergyChange(headDisplacement, HeadMass, lastTwoBodies);

            Vector3D torsoDisplacement = CalculateDisplacementTorso(lastTwoBodies);
            totalPotential += CalculatePotentialEnergyChange(torsoDisplacement, TorsoMass);
            totalKinetic += CalculateKineticEnergyChange(torsoDisplacement, TorsoMass, lastTwoBodies);

            if (handTracking)
            {
                Vector3D[] handDisplacement = CalculateDisplacementHands(lastTwoBodies);
                totalPotential += CalculatePotentialEnergyChange(handDisplacement[0], ForeArmMass);
                totalKinetic += CalculateKineticEnergyChange(handDisplacement[0], ForeArmMass, lastTwoBodies);
                totalPotential += CalculatePotentialEnergyChange(handDisplacement[1], ForeArmMass);
                totalKinetic += CalculateKineticEnergyChange(handDisplacement[1], ForeArmMass, lastTwoBodies);
            }

            body.EnergyLevel = totalKinetic + totalPotential;
        }

        private static Vector3D CalculateDisplacementHead(Body[] bodies)
        {
            float displacementX = 0;
            float displacementY = 0;
            float displacementZ = 0;

            displacementX += Math.Abs(bodies[0].Head.AvgCenterPoint.X - bodies[1].Head.AvgCenterPoint.X);
            displacementY += Math.Abs(bodies[0].Head.AvgCenterPoint.Y - bodies[1].Head.AvgCenterPoint.Y);
            displacementZ += Math.Abs(bodies[0].Head.AvgCenterPoint.Z - bodies[1].Head.AvgCenterPoint.Z);

            return new Vector3D(displacementX, displacementY, displacementZ);
        }

        private static Vector3D CalculateDisplacementTorso(Body[] bodies)
        {
            float displacementX = 0;
            float displacementY = 0;
            float displacementZ = 0;

            for (int i = 1; i < bodies.Length; i++)
            {
                if (bodies[i].Torso != null && bodies[i-1].Torso != null && !float.IsNaN(bodies[i].Torso.AvgCenterPoint.X) && !float.IsNaN(bodies[i - 1].Torso.AvgCenterPoint.X))
                {
                    displacementX += Math.Abs(bodies[i].Torso.AvgCenterPoint.X - bodies[i - 1].Torso.AvgCenterPoint.X);
                    displacementY += Math.Abs(bodies[i].Torso.AvgCenterPoint.Y - bodies[i - 1].Torso.AvgCenterPoint.Y);
                    displacementZ += Math.Abs(bodies[i].Torso.AvgCenterPoint.Z - bodies[i - 1].Torso.AvgCenterPoint.Z);

                }
            }

            return new Vector3D(displacementX, displacementY, displacementZ);
        }

        private static Vector3D[] CalculateDisplacementHands(Body[] bodies)
        {
            float displacementFirstHandX = 0;
            float displacementFirstHandY = 0;
            float displacementFirstHandZ = 0;

            float displacementSecondHandX = 0;
            float displacementSecondHandY = 0;
            float displacementSecondHandZ = 0;

            for (int i = 1; i < bodies.Length; i++)
            {
                if (bodies[i].Hands[0] != null && bodies[i - 1].Hands[0] != null && !float.IsNaN(bodies[i].Hands[0].AvgCenterPointLastFrames.X) && !float.IsNaN(bodies[i - 1].Hands[0].AvgCenterPointLastFrames.X))
                {
                    displacementFirstHandX = Math.Abs(bodies[i].Hands[0].AvgCenterPointLastFrames.X - bodies[i - 1].Hands[0].AvgCenterPointLastFrames.X);
                    displacementFirstHandY = Math.Abs(bodies[i].Hands[0].AvgCenterPointLastFrames.Y - bodies[i - 1].Hands[0].AvgCenterPointLastFrames.Y);
                    displacementFirstHandZ = Math.Abs(bodies[i].Hands[0].AvgCenterPointLastFrames.Z - bodies[i - 1].Hands[0].AvgCenterPointLastFrames.Z); 
                }
                if (bodies[i].Hands[1] != null && bodies[i - 1].Hands[1] != null && !float.IsNaN(bodies[i].Hands[1].AvgCenterPointLastFrames.X) && !float.IsNaN(bodies[i - 1].Hands[1].AvgCenterPointLastFrames.X))
                {
                    displacementSecondHandX = Math.Abs(bodies[i].Hands[1].AvgCenterPointLastFrames.X - bodies[i - 1].Hands[1].AvgCenterPointLastFrames.X);
                    displacementSecondHandY = Math.Abs(bodies[i].Hands[1].AvgCenterPointLastFrames.Y - bodies[i - 1].Hands[1].AvgCenterPointLastFrames.Y);
                    displacementSecondHandZ = Math.Abs(bodies[i].Hands[1].AvgCenterPointLastFrames.Z - bodies[i - 1].Hands[1].AvgCenterPointLastFrames.Z);
                }
            }
            var displacementFirstHand = new Vector3D(displacementFirstHandX,displacementFirstHandY,displacementFirstHandZ);
            var displacementSecondHand  = new Vector3D(displacementSecondHandX,displacementSecondHandY,displacementSecondHandZ);

            return new[]{displacementFirstHand,displacementSecondHand};
        }

        private static double CalculatePotentialEnergyChange(Vector3D displacement, double mass)
        {
            if (displacement.Z <= 0 || double.IsNaN(displacement.Z))
            {
                return 0;
            }

            return mass*Gravity*displacement.Z;
        }

        private static double CalculateKineticEnergyChange(Vector3D displacement, double mass, Body[] bodies)
        {
            if (double.IsNaN(displacement.Z))
            {
                return 0;
            }

            TimeSpan timeStampStart = bodies[0].TimeStamp;
            TimeSpan timeStampStop = bodies[1].TimeStamp;
            double deltaTime = timeStampStop.Subtract(timeStampStart).TotalSeconds;

            if (deltaTime == 0)
            {
                return 0;
            }

            double velocityX = displacement.X/deltaTime;
            double velocityY = displacement.Y/deltaTime;

            double kineticEnergyX = 0.5*mass*velocityX*velocityX;
            double kineticEnergyY = 0.5*mass*velocityY*velocityY;

            return kineticEnergyX + kineticEnergyY;
        }

    }
}
