package us.ihmc.manipulation.planning.manipulation.solarpanelmotion;

import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage.BaseForControl;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;
import us.ihmc.robotics.robotSide.RobotSide;

public class SolarPanelStraightPathPlanner
{
   private SolarPanelStraightPath path;
   private HandTrajectoryMessage handTrajectoryMessage;                 // only yaw angle I can
   private PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage; // temporary constant
   private ChestTrajectoryMessage chestTrajectoryMessage;               // temporary constant
      
   public SolarPanelStraightPathPlanner(SolarPanelStraightPath solarPanelStraightPath)
   {
      path = solarPanelStraightPath;
      handTrajectoryMessage = new HandTrajectoryMessage(RobotSide.RIGHT, BaseForControl.WORLD, path.getPoses().size());
      // pelvisheight
      // chest
   }
   
   // select random start yaw
   // select random end yaw
   // interpolate Linealy
   
   // set node, discrete time and yaw
}
