package us.ihmc.manipulation.planning.manipulation.solarpanelmotion;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.robotics.geometry.transformables.Pose;
import us.ihmc.robotics.robotSide.RobotSide;

public class SolarPanelMotionPlanner
{
   private SolarPanel solarPanel;
   
   WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();   
   
   // debug
   public Pose debugPoseOne;
   public Pose debugPoseTwo;
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
      RRTPlanningMotion
   }
   
   public WholeBodyTrajectoryMessage getWholeBodyTrajectoryMessage()
   {
      return this.wholeBodyTrajectoryMessage;
   }
   
//   public void setWholeBodyTrajectoryMessage(CleaningMotion motion, double motionTime)
//   {
//      readyPose = new SolarPanelCleaningPose(solarPanel, 0.5, 0.1, -0.1, -Math.PI*0.2);
//      
//      this.wholeBodyTrajectoryMessage.clear();
//           
//      switch(motion)
//      {
//      case ReadyPose:
//         PrintTools.info("setTrajectoryMessage -> "+CleaningMotion.ReadyPose);
//         
//         wholeBodyTrajectoryMessage.setHandTrajectoryMessage(readyPose.getHandTrajectoryMessage(motionTime));
//         debugPoseOne = readyPose.getPose();
//         
//         break;
//      case LinearCleaningMotion:
//         PrintTools.info("setTrajectoryMessage -> "+CleaningMotion.LinearCleaningMotion);
//         
//         SolarPanelCleaningPose lineStart = new SolarPanelCleaningPose(readyPose);
//         SolarPanelCleaningPose lineEnd = new SolarPanelCleaningPose(solarPanel, 0.1, 0.1, -0.1);
//         SolarPanelStraightPath linePath = new SolarPanelStraightPath(solarPanel, motionTime, lineStart, lineEnd, 10);
//         SolarPanelStraightPathPlanner cleaningMotionPlanner = new SolarPanelStraightPathPlanner(linePath);
//         
//         cleaningMotionPlanner.temporarySolution();
//         wholeBodyTrajectoryMessage.setHandTrajectoryMessage(cleaningMotionPlanner.getHandTrajectoryTemporary());         
//         wholeBodyTrajectoryMessage.setChestTrajectoryMessage(cleaningMotionPlanner.getChestTrajectoryTemporary());
//         
//         
//         debugPoseOne = linePath.getStartPose().getPose();
//         debugPoseTwo = linePath.getEndPose().getPose();
//         
//         break;
//         
//      case LinearCleaningMotionWhole:
//         
//         ArrayList<SolarPanelCleaningPose> tempPoses = new ArrayList<SolarPanelCleaningPose>();
//         
//         // ten
//         tempPoses.add(new SolarPanelCleaningPose(readyPose));
//         tempPoses.add(new SolarPanelCleaningPose(solarPanel, 0.1, 0.1, -0.1));
//         tempPoses.add(new SolarPanelCleaningPose(solarPanel, 0.1, 0.2, -0.1));
//         tempPoses.add(new SolarPanelCleaningPose(solarPanel, 0.5, 0.2, -0.1));
//         tempPoses.add(new SolarPanelCleaningPose(solarPanel, 0.5, 0.3, -0.1));
//         tempPoses.add(new SolarPanelCleaningPose(solarPanel, 0.1, 0.3, -0.1));
//         tempPoses.add(new SolarPanelCleaningPose(solarPanel, 0.1, 0.4, -0.1));
//         tempPoses.add(new SolarPanelCleaningPose(solarPanel, 0.5, 0.4, -0.1));
//         tempPoses.add(new SolarPanelCleaningPose(solarPanel, 0.5, 0.5, -0.1));
//         tempPoses.add(new SolarPanelCleaningPose(solarPanel, 0.1, 0.5, -0.1));
//         
//         ArrayList<SolarPanelStraightPath> paths = new ArrayList<SolarPanelStraightPath>();
//         
//         for(int i=0;i<tempPoses.size()-1;i++)
//         {
//            paths.add(new SolarPanelStraightPath(solarPanel, motionTime, tempPoses.get(i), tempPoses.get(i+1), 3));            
//         }
//         SolarPanelStraightPathPlanner cleaningMotionPlanner1 = new SolarPanelStraightPathPlanner(paths);
//         cleaningMotionPlanner1.temporarySolution();
//         
//         for(int i=0;i<tempPoses.size();i++)
//         {
//            debugPose.add(tempPoses.get(i).getPose());            
//         }
//                  
//         wholeBodyTrajectoryMessage.setHandTrajectoryMessage(cleaningMotionPlanner1.getHandTrajectoryTemporaryWhole());         
//         wholeBodyTrajectoryMessage.setChestTrajectoryMessage(cleaningMotionPlanner1.getChestTrajectoryTemporaryWhole());
//         
//         
//         
//         break;
//         
//      default:
//         PrintTools.info("setTrajectoryMessage -> NONE");
//         
//         break;
//         
//      }
//      
//   }
   
   public void setWholeBodyTrajectoryMessage(CleaningMotion motion)
   {
      WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();
      
      SolarPanelCleaningPose readyPose = new SolarPanelCleaningPose(solarPanel, 0.5, 0.1, -0.1, -Math.PI*0.2);
      
      
           
      switch(motion)
      {
      case ReadyPose:
         PrintTools.info("setTrajectoryMessage -> "+CleaningMotion.ReadyPose);
         SolarPanelWholeBodyPose wholebodyReadyPose = new SolarPanelWholeBodyPose(readyPose, 0.0, 0.0);
         wholebodyReadyPose.getWholeBodyTrajectoryMessage(wholeBodyTrajectoryMessage, 3.0);
         
         debugPoseOne = readyPose.getPose();
         
         break;
      case LinearCleaningMotion:
         PrintTools.info("setTrajectoryMessage -> "+CleaningMotion.LinearCleaningMotion);
         
         
         break;
         
      case LinearCleaningMotionWhole:
         
         
         
         
         break;
         
      default:
         PrintTools.info("setTrajectoryMessage -> NONE");
         
         break;
         
      }
      
      this.wholeBodyTrajectoryMessage = wholeBodyTrajectoryMessage;
   }
   
   public double getMotionTime()
   {
      PrintTools.info("MotionTime is "+this.wholeBodyTrajectoryMessage.getHandTrajectoryMessage(RobotSide.RIGHT).getTrajectoryTime());
      return this.wholeBodyTrajectoryMessage.getHandTrajectoryMessage(RobotSide.RIGHT).getTrajectoryTime();
   }
   
   
   
   
   
   
}
