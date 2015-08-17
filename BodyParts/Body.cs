//
// Copyright (c) Leif Erik Bjoerkli, Norwegian University of Science and Technology, 2015.
// Distributed under the MIT License.
// (See accompanying file LICENSE or copy at http://opensource.org/licenses/MIT)
//  

using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Kinect;

namespace Kinect2TrackingTopView
{
    public class Body
    {
        public Hand[] Hands { get; private set; }
        public Head Head { get; private set; }
        public Torso Torso { get; private set; }
        public int Id { get; private set; }
        public TimeSpan TimeStamp { get; private set; }
        public double EnergyLevel { get; set; }

        public Body(int id, TimeSpan timeStamp)
        {
            Id = id;
            Hands = new Hand[2];
            TimeStamp = timeStamp;
        }

        public void AddHead(Head head)
        {

            Head = head;
        }

        /// <summary>
        /// Adds the handpoints to the body. 
        /// </summary>
        /// /// <remarks> 
        /// The new candidate hands are compared to average location of previous hands for bodies with same id, to identify if the candidate hands are left or right.
        /// </remarks> 
        public void AddHands(List<int> handPoints)
        {
            List<CameraSpacePoint> handCenterPoints = BodyUtils.GroupHandPoints(handPoints);

            Hand[] newHands = new Hand[2];

            var averageHandLocationsLastFrames = BodyUtils.GetAverageHandLocationsLastFrames(Id);
            CameraSpacePoint avgFirstHandPoint = averageHandLocationsLastFrames[0];
            CameraSpacePoint avgSecondHandPoint = averageHandLocationsLastFrames[1];

            var distancesToHandAverages = BodyUtils.GetDistancesToHandAveragesLastFrames(handCenterPoints, avgFirstHandPoint, avgSecondHandPoint);

            var sortedDistancesToHandAverages = distancesToHandAverages.OrderBy(kvp => kvp.Value);

            if (BodyUtils.HasFirstHandTracking(Id) || BodyUtils.HasSecondHandTracking(Id))
            {
                bool[] handsUsed = { false, false };

                foreach (var distance in sortedDistancesToHandAverages)
                {
                    if (distance.Key == 0)
                    {
                        if (BodyUtils.HasFirstHandTracking(Id))
                        {
                            if (distancesToHandAverages[distance.Key] < Thresholds.LastFramesHandMaxDistance)
                            {
                                Hand hand = new Hand(handCenterPoints[0], 0, avgFirstHandPoint);
                                if (newHands[0] == null && !handsUsed[0])
                                {
                                    newHands[0] = hand;
                                    handsUsed[0] = true;
                                }
                            }
                        }
                        else
                        {
                            Hand hand = new Hand(handCenterPoints[0], 0, avgFirstHandPoint);
                            if (newHands[0] == null && !handsUsed[0])
                            {
                                newHands[0] = hand;
                                handsUsed[0] = true;
                            }
                        }
                    }
                    else if (distance.Key == 1)
                    {
                        if (BodyUtils.HasSecondHandTracking(Id))
                        {
                            if (distancesToHandAverages[distance.Key] < Thresholds.LastFramesHandMaxDistance)
                            {
                                Hand hand = new Hand(handCenterPoints[0], 1, avgSecondHandPoint);
                                if (newHands[1] == null && !handsUsed[0])
                                {
                                    newHands[1] = hand;
                                    handsUsed[0] = true;
                                }
                            }
                        }
                        else
                        {
                            Hand hand = new Hand(handCenterPoints[0], 1, avgSecondHandPoint);
                            if (newHands[1] == null && !handsUsed[0])
                            {
                                newHands[1] = hand;
                                handsUsed[0] = true;
                            }
                        }
                    }
                    else if (distance.Key == 2)
                    {
                        if (BodyUtils.HasFirstHandTracking(Id))
                        {
                            if (distancesToHandAverages[distance.Key] < Thresholds.LastFramesHandMaxDistance)
                            {
                                Hand hand = new Hand(handCenterPoints[1], 0, avgFirstHandPoint);
                                if (newHands[0] == null && !handsUsed[1])
                                {
                                    newHands[0] = hand;
                                    handsUsed[1] = true;
                                }
                            }
                        }
                        else
                        {
                            Hand hand = new Hand(handCenterPoints[1], 0, avgFirstHandPoint);
                            if (newHands[0] == null && !handsUsed[1])
                            {
                                newHands[0] = hand;
                                handsUsed[1] = true;
                            }
                        }
                    }
                    else if (distance.Key == 3)
                    {
                        if (BodyUtils.HasSecondHandTracking(Id))
                        {
                            if (distancesToHandAverages[distance.Key] < Thresholds.LastFramesHandMaxDistance)
                            {
                                Hand hand = new Hand(handCenterPoints[1], 1, avgSecondHandPoint);
                                if (newHands[1] == null && !handsUsed[1])
                                {
                                    newHands[1] = hand;
                                    handsUsed[1] = true;
                                }
                            }
                        }
                        else
                        {
                            Hand hand = new Hand(handCenterPoints[1], 1, avgSecondHandPoint);
                            if (newHands[1] == null && !handsUsed[1])
                            {
                                newHands[1] = hand;
                                handsUsed[1] = true;
                            }
                        }
                    }
                }
            }
            else
            {
                for (int i = 0; i < handCenterPoints.Count; i++)
                {
                    newHands[i] = new Hand(handCenterPoints[i], i, new CameraSpacePoint() { X = float.NaN, Y = float.NaN, Z = float.NaN });
                }
            }
            Hands = newHands;
        }
        
        /// <summary>
        /// Adds the torsopoints to body
        /// </summary>
        /// /// <remarks> 
        /// If the current body id had tracking in previous frames, the average torso location the last frames is saved. 
        /// </remarks> 
        public void AddTorso(List<int> torsoPoints)
        {
            CameraSpacePoint currentAvg = BodyUtils.CalculateAveragePointFromValidPoints(torsoPoints);
            if (BodyUtils.HasTorsoTracking(Id))
            {
                CameraSpacePoint avgLastFrames = BodyUtils.GetAverageTorsoLocationLastFrames(Id);

                if (GlobUtils.GetEuclideanDistance(avgLastFrames, currentAvg) < Thresholds.LastFramesTorsoMaxDistance)
                {
                    Torso torso = new Torso(currentAvg, avgLastFrames);
                    Torso = torso;
                }
                else
                {
                    Torso torso = new Torso(currentAvg, new CameraSpacePoint() { X = float.NaN, Y = float.NaN, Z = float.NaN });
                    Torso = torso;    
                }
            }
            else
            {
                Torso torso = new Torso(currentAvg, new CameraSpacePoint() { X = float.NaN, Y = float.NaN, Z = float.NaN });
                Torso = torso;
            }

        }


    }
}
