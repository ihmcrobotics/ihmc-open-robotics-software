package us.ihmc.manipulation.planning.manipulation.solarpanelmotion;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanel;
import us.ihmc.robotics.geometry.transformables.Pose;
import us.ihmc.robotics.robotSide.RobotSide;

public class SolarPanelMotionPlanner
{
   private SolarPanel solarPanel;
   
   WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();   
   
   // For debug
   public ArrayList<Pose> debugPose = new ArrayList<Pose>();
   
   public SolarPanelMotionPlanner(SolarPanel panel)
   {
      this.solarPanel = panel;
   }
   
   public enum CleaningMotion
   {
      ReadyPose,
      LinearCleaningMotion,
      LinearCleaningMotionWhole,
      RRTTimeDomainLinearMotion,
      RRTTimeDomainLinearWhole,
   }
   
   public WholeBodyTrajectoryMessage getWholeBodyTrajectoryMessage()
   {
      return this.wholeBodyTrajectoryMessage;
   }   
      
   public boolean setWholeBodyTrajectoryMessage(CleaningMotion motion)
   {
      WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();
      
      SolarPanelCleaningPose readyPose = new SolarPanelCleaningPose(solarPanel, 0.5, 0.1, -0.1, -Math.PI*0.2);
      
      
           
      switch(motion)
      {
      case ReadyPose:
         PrintTools.info("setTrajectoryMessage -> "+CleaningMotion.ReadyPose);
         SolarPanelWholeBodyPose wholebodyReadyPose = new SolarPanelWholeBodyPose(readyPose, 0.0, 0.0);
         wholebodyReadyPose.getWholeBodyTrajectoryMessage(wholeBodyTrajectoryMessage, 3.0);
         
         debugPose.add(readyPose.getPose());
         
         break;
         
      case LinearCleaningMotion:
         PrintTools.info("setTrajectoryMessage -> "+CleaningMotion.LinearCleaningMotion);
         SolarPanelPath cleaningPath = new SolarPanelPath(readyPose);
         
         SolarPanelCleaningPose endPose = new SolarPanelCleaningPose(solarPanel, 0.5, 0.1, -0.1, -Math.PI*0.2);         
         
         cleaningPath.addCleaningPose(endPose, 4.0);
         
         // Following line should have an error.
         wholeBodyTrajectoryMessage = cleaningPath.getWholeBodyMessage();
         
         break;
         
      case LinearCleaningMotionWhole:
         
         
         
         
         break;
         
      case RRTTimeDomainLinearMotion:
         
         
         
         
         break;
         
      case RRTTimeDomainLinearWhole:
         
         
         
         
         break;
         
      default:
         PrintTools.info("setTrajectoryMessage -> NONE");
         
         break;
         
      }
      
      this.wholeBodyTrajectoryMessage.clear();
      this.wholeBodyTrajectoryMessage = wholeBodyTrajectoryMessage;
      
      return true;
   }
   
   public double getMotionTime()
   {
      PrintTools.info("MotionTime is "+this.wholeBodyTrajectoryMessage.getHandTrajectoryMessage(RobotSide.RIGHT).getTrajectoryTime());
      return this.wholeBodyTrajectoryMessage.getHandTrajectoryMessage(RobotSide.RIGHT).getTrajectoryTime();
   }
   
   
   
   
   
   
}
