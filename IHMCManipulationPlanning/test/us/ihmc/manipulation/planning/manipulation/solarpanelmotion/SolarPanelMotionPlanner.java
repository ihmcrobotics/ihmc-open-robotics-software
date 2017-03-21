package us.ihmc.manipulation.planning.manipulation.solarpanelmotion;

import us.ihmc.commons.PrintTools;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;

public class SolarPanelMotionPlanner
{
   private SolarPanel solarPanel;
   
   WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();   
   private SolarPanelStraightPathHandTrajectory linearTrajectory;
   
   public SolarPanelMotionPlanner(SolarPanel panel)
   {
      this.solarPanel = panel;
   }
   
   public enum CleaningMotion
   {
      ReadyPose,
      TestCleaningMotion
   }
   
   public WholeBodyTrajectoryMessage getWholeBodyTrajectoryMessage()
   {
      return this.wholeBodyTrajectoryMessage;
   }
   
   public void setWholeBodyTrajectoryMessage(CleaningMotion motion)
   {
      this.wholeBodyTrajectoryMessage.clear();
      switch(motion)
      {
      case ReadyPose:
         PrintTools.info("setTrajectoryMessage -> "+CleaningMotion.ReadyPose);
         
         break;
      case TestCleaningMotion:
         PrintTools.info("setTrajectoryMessage -> "+CleaningMotion.TestCleaningMotion);
         
         break;
         
      }
      
   }
   
   
   
   
   
   
}
