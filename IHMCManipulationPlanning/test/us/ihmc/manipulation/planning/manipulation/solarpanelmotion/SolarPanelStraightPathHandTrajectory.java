package us.ihmc.manipulation.planning.manipulation.solarpanelmotion;

import java.util.ArrayList;

import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;

public class SolarPanelStraightPathHandTrajectory extends SolarPanelStraightPath
{
   private ArrayList<HandTrajectoryMessage> handTrajectoryMessages = new ArrayList<HandTrajectoryMessage>();
   
   public SolarPanelStraightPathHandTrajectory(SolarPanel solarPanel, SolarPanelCleaningPose startPose, SolarPanelCleaningPose endPose, int numberOfWayPoints)
   {
      super(solarPanel, startPose, endPose, numberOfWayPoints);
   }
      
   public ArrayList<HandTrajectoryMessage> getHandTrajectoryMessages()
   {
      return handTrajectoryMessages;
   }
}
