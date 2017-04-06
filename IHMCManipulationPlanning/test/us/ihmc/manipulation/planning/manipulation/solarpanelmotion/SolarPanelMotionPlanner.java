package us.ihmc.manipulation.planning.manipulation.solarpanelmotion;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.manipulation.planning.rrttimedomain.RRTNode1DTimeDomain;
import us.ihmc.manipulation.planning.rrttimedomain.RRTPlanner1DTimeDomain;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanel;
import us.ihmc.robotics.geometry.transformables.Pose;
import us.ihmc.robotics.robotSide.RobotSide;

public class SolarPanelMotionPlanner
{
   private SolarPanel solarPanel;
   
   WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();   
   
   public RRTPlanner1DTimeDomain plannerTimeDomain;
   
   // For debug
   public ArrayList<Pose> debugPose = new ArrayList<Pose>();
   
   public SolarPanelMotionPlanner(SolarPanel panel)
   {
      this.solarPanel = panel;
      
      RRTNode1DTimeDomain nodeRoot = new RRTNode1DTimeDomain(0.0, 0.0);
      plannerTimeDomain = new RRTPlanner1DTimeDomain(nodeRoot);
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
         
         // *** RRT *** //
         
         double motionTime = 5.0;
         
         RRTNode1DTimeDomain nodeLowerBound = new RRTNode1DTimeDomain(0.0, -Math.PI*0.4);
         RRTNode1DTimeDomain nodeUpperBound = new RRTNode1DTimeDomain(motionTime*1.5, Math.PI*0.4);
         
         plannerTimeDomain.getTree().setMotionTime(motionTime);
         plannerTimeDomain.getTree().setUpperBound(nodeUpperBound);
         plannerTimeDomain.getTree().setLowerBound(nodeLowerBound);
         
         RRTNode1DTimeDomain.cleaningPath = cleaningPath;
         plannerTimeDomain.expandTreeGoal(500);
         //plannerTimeDomain.updateOptimalPath(101, 100);
         

         
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
