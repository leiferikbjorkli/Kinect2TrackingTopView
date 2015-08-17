//
// Written by Leif Erik Bjoerkli
//


using System;
using System.Collections.Generic;
using System.Linq;
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

        private List<Dictionary<int, double>> energyHistory;
        private List<TimeSpan> relativeTimeHistory;

        // Head moves up with body kinetic and potential energy

        // Total energy expenditure


        public EnergyUtils()
        {
            energyHistory = new List<Dictionary<int, double>>();
            relativeTimeHistory = new List<TimeSpan>();
        }

        public List<Dictionary<int, double>> GetEnergyHistory()
        {
            var filteredHistory = FilterNoise(energyHistory);
            var smoothedHistory = MedianOneDimension(filteredHistory);

            return smoothedHistory;
        }

        private List<Dictionary<int, double>> FilterNoise(List<Dictionary<int, double>> energyHistory)
        {
            var mostTrackedBodyId = GetMostTrackedBodyId(energyHistory);

            var filteredEnergyHistory = new List<Dictionary<int,double>>();

            foreach (var frame in energyHistory)
            {
                var filteredFrame = new Dictionary<int,double>();
                foreach (var energy in frame)
                {
                    if (energy.Key == mostTrackedBodyId)
                    {
                        filteredFrame.Add(1,energy.Value);
                    }
                }
                filteredEnergyHistory.Add(filteredFrame);
            }

            return filteredEnergyHistory;
        }

        private int GetMostTrackedBodyId(List<Dictionary<int, double>> energyHistory)
        {
            var count = new Dictionary<int, int>();

            for (int i = 1; i <= GlobVar.MaxBodyCount; i++)
            {
                count.Add(i, 0);
            }

            foreach (var frame in energyHistory)
            {
                foreach (var energy in frame)
                {
                    count[energy.Key]++;
                }
            }

            var sortedCount = count.OrderByDescending(kvp => kvp.Value);

            return sortedCount.ElementAt(0).Key;
        }

        private List<Dictionary<int, double>> MedianOneDimension(List<Dictionary<int, double>> energyHistory)
        {
            var filteredHistory = new List<Dictionary<int, double>>();


            filteredHistory.Add(new Dictionary<int, double>());
            filteredHistory.Add(new Dictionary<int, double>());
            for (int i = 2; i < energyHistory.Count-2; i++)
            {
                var filteredFrame = new Dictionary<int,double>();

                foreach (var frameEnergy in energyHistory[i])
                {
                    var energyKernel = new double[5];

                    int bodyId = frameEnergy.Key;

                    double energyFrame1 = 0;
                    energyHistory[i - 2].TryGetValue(bodyId, out energyFrame1);
                    energyKernel[0] = energyFrame1;

                    double energyFrame2 = 0;
                    energyHistory[i - 1].TryGetValue(bodyId,out energyFrame2);
                    energyKernel[1] = energyFrame2;

                    double energyFrame3 = 0;
                    energyHistory[i].TryGetValue(bodyId, out energyFrame3);
                    energyKernel[2] = energyFrame3;

                    double energyFrame4 = 0;
                    energyHistory[i + 1].TryGetValue(bodyId, out energyFrame4);
                    energyKernel[3] = energyFrame4;

                    double energyFrame5 = 0;
                    energyHistory[i + 2].TryGetValue(bodyId, out energyFrame5);
                    energyKernel[4] = energyFrame5;

                    Array.Sort(energyKernel);
                    filteredFrame.Add(bodyId,energyKernel[2]);
                }
                filteredHistory.Add(filteredFrame);
            }
            filteredHistory.Add(new Dictionary<int, double>());
            filteredHistory.Add(new Dictionary<int, double>());

            return filteredHistory;
        }

        public List<TimeSpan> RelativeTimeHistory
        {
            get { return relativeTimeHistory; }
        }

        public void UpdateGlobalEnergy(List<Body> bodies, TimeSpan timestamp)
        {
            var currentFrameEnergy = new Dictionary<int,double>();
            foreach (var body in bodies)
            {
                currentFrameEnergy.Add(body.Id,body.EnergyLevel);
            }

            energyHistory.Add(currentFrameEnergy);
            RelativeTimeHistory.Add(timestamp);
        }

        public static void UpdateBodyEnergy(Body body)
        {

            List<Body> lastBodies = BodyUtils.GetBodiesFromHistory(body.Id);

            if (lastBodies.Count < 2)
            {
                body.EnergyLevel = 0;
                return;
            }

            Body[] lastTwoBodies = new Body[2];
            lastTwoBodies[0] = (lastBodies[lastBodies.Count - 2]);
            lastTwoBodies[1] = (lastBodies[lastBodies.Count - 1]); ;

            double totalPotential = 0;
            double totalKinetic = 0;

            Vector3D headDisplacement = CalculateDisplacementHeadTest(lastTwoBodies); 
            totalPotential += CalculatePotentialEnergyChange(headDisplacement, HeadMass);
            totalKinetic += CalculateKineticEnergyChange(headDisplacement,HeadMass, lastTwoBodies);

            Vector3D torsoDisplacement = CalculateDisplacementTorso(lastTwoBodies);
            totalPotential += CalculatePotentialEnergyChange(torsoDisplacement, TorsoMass);
            totalKinetic += CalculateKineticEnergyChange(torsoDisplacement, TorsoMass, lastTwoBodies);

            if (torsoDisplacement.Z > 0.2f || torsoDisplacement.X > 1f || torsoDisplacement.Y > 1f)
            {

            }

            //Vector3D[] handDisplacement =
            //CalculateDisplacementHands(lastTwoBodies); totalPotential +=
            //CalculatePotentialEnergyChange(handDisplacement[0], ForeArmMass);
            //totalKinetic += CalculateKineticEnergyChange(handDisplacement[0],
            //ForeArmMass,lastTwoBodies); totalPotential +=
            //CalculatePotentialEnergyChange(handDisplacement[1], ForeArmMass);
            //totalKinetic += CalculateKineticEnergyChange(handDisplacement[1],
            //ForeArmMass, lastTwoBodies);

            if ((totalKinetic + totalPotential) > 50)
            {
                
            }

            body.EnergyLevel = totalKinetic + totalPotential;
        }
        private static Vector3D CalculateDisplacementHeadTest(Body[] bodies)
        {
            float displacementX = 0;
            float displacementY = 0;
            float displacementZ = 0;

            displacementX += Math.Abs(bodies[0].Head.AvgCenterPoint.X - bodies[1].Head.AvgCenterPoint.X);
            displacementY += Math.Abs(bodies[0].Head.AvgCenterPoint.Y - bodies[1].Head.AvgCenterPoint.Y);
            displacementZ += Math.Abs(bodies[0].Head.AvgCenterPoint.Z - bodies[1].Head.AvgCenterPoint.Z);

            return new Vector3D(displacementX, displacementY, displacementZ);
        }

        private static Vector3D CalculateDisplacementHead(Body[] bodies)
        {
            float displacementX = 0;
            float displacementY = 0;
            float displacementZ = 0;

            for (int i = 1; i < bodies.Length; i++)
            {
                displacementX += Math.Abs(bodies[i].Head.CenterPoint.X - bodies[i - 1].Head.AvgCenterPoint.X);
                displacementY += Math.Abs(bodies[i].Head.CenterPoint.Y - bodies[i - 1].Head.AvgCenterPoint.Y);
                displacementZ += Math.Abs(bodies[i].Head.CenterPoint.Z - bodies[i - 1].Head.AvgCenterPoint.Z);
            }

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
                if (bodies[i].Hands[0] != null && bodies[i - 1].Hands[0] != null && !float.IsNaN(bodies[i].Hands[0].AvgCenterPointLastFiveFrames.X) && !float.IsNaN(bodies[i - 1].Hands[0].AvgCenterPointLastFiveFrames.X))
                {
                    displacementFirstHandX = Math.Abs(bodies[i].Hands[0].AvgCenterPointLastFiveFrames.X - bodies[i - 1].Hands[0].AvgCenterPointLastFiveFrames.X);
                    displacementFirstHandY = Math.Abs(bodies[i].Hands[0].AvgCenterPointLastFiveFrames.Y - bodies[i - 1].Hands[0].AvgCenterPointLastFiveFrames.Y);
                    displacementFirstHandZ = Math.Abs(bodies[i].Hands[0].AvgCenterPointLastFiveFrames.Z - bodies[i - 1].Hands[0].AvgCenterPointLastFiveFrames.Z); 
                }
                if (bodies[i].Hands[1] != null && bodies[i - 1].Hands[1] != null && !float.IsNaN(bodies[i].Hands[1].AvgCenterPointLastFiveFrames.X) && !float.IsNaN(bodies[i - 1].Hands[1].AvgCenterPointLastFiveFrames.X))
                {
                    displacementSecondHandX = Math.Abs(bodies[i].Hands[1].AvgCenterPointLastFiveFrames.X - bodies[i - 1].Hands[1].AvgCenterPointLastFiveFrames.X);
                    displacementSecondHandY = Math.Abs(bodies[i].Hands[1].AvgCenterPointLastFiveFrames.Y - bodies[i - 1].Hands[1].AvgCenterPointLastFiveFrames.Y);
                    displacementSecondHandZ = Math.Abs(bodies[i].Hands[1].AvgCenterPointLastFiveFrames.Z - bodies[i - 1].Hands[1].AvgCenterPointLastFiveFrames.Z);
                }
            }
            var displacementFirstHand = new Vector3D(displacementFirstHandX,displacementFirstHandY,displacementFirstHandZ);
            var displacementSecondHand  = new Vector3D(displacementSecondHandX,displacementSecondHandY,displacementSecondHandZ);

            return new Vector3D[]{displacementFirstHand,displacementSecondHand};
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
