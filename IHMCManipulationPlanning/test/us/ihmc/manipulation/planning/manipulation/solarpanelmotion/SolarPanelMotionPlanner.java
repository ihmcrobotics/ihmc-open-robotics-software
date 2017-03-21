package us.ihmc.manipulation.planning.manipulation.solarpanelmotion;

import us.ihmc.commons.PrintTools;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.robotics.geometry.transformables.Pose;

public class SolarPanelMotionPlanner
{
   private SolarPanel solarPanel;
   
   WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();   
   SolarPanelCleaningPose readyPose;
   
   // debug
   public Pose debugPoseOne;
   public Pose debugPoseTwo;
   
   public SolarPanelMotionPlanner(SolarPanel panel)
   {
      this.solarPanel = panel;
   }
   
   public enum CleaningMotion
   {
      ReadyPose,
      LinearCleaningMotion,
      RRTPlanningMotion
   }
   
   public WholeBodyTrajectoryMessage getWholeBodyTrajectoryMessage()
   {
      return this.wholeBodyTrajectoryMessage;
   }
   
   public void setWholeBodyTrajectoryMessage(CleaningMotion motion, double motionTime)
   {
      readyPose = new SolarPanelCleaningPose(solarPanel, 0.5, 0.1, -0.1, -Math.PI*0.2);
      
      this.wholeBodyTrajectoryMessage.clear();
           
      switch(motion)
      {
      case ReadyPose:
         PrintTools.info("setTrajectoryMessage -> "+CleaningMotion.ReadyPose);
         
         wholeBodyTrajectoryMessage.setHandTrajectoryMessage(readyPose.getHandTrajectoryMessage(motionTime));
         
         break;
      case LinearCleaningMotion:
         PrintTools.info("setTrajectoryMessage -> "+CleaningMotion.LinearCleaningMotion);
         
         SolarPanelCleaningPose lineStart = new SolarPanelCleaningPose(readyPose);
         SolarPanelCleaningPose lineEnd = new SolarPanelCleaningPose(solarPanel, 0.1, 0.1, -0.1);
         SolarPanelStraightPath linePath = new SolarPanelStraightPath(solarPanel, lineStart, lineEnd, 10);
         
         SolarPanelStraightPathPlanner cleaningMotionPlanner = new SolarPanelStraightPathPlanner(linePath, motionTime);
         cleaningMotionPlanner.getOptimalRedundancyForSLIR();
         wholeBodyTrajectoryMessage.setHandTrajectoryMessage(cleaningMotionPlanner.getHandTrajectorySLIR());
         wholeBodyTrajectoryMessage.setChestTrajectoryMessage(cleaningMotionPlanner.getChestTrajectorySLIR());
         
         debugPoseOne = linePath.getStartPose().getPose();
         debugPoseTwo = linePath.getEndPose().getPose();
         
         break;
         
      default:
         PrintTools.info("setTrajectoryMessage -> NONE");
         
         break;
         
      }
      
   }
   
   
   
   
   
   
}
