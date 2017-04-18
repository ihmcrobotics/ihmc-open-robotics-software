package us.ihmc.humanoidBehaviors.behaviors.solarPanel;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanel;
import us.ihmc.robotics.geometry.transformables.Pose;

public class SolarPanelMotionPlanner
{
   private SolarPanel solarPanel;
   
   WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();   
   SolarPanelWholeBodyTrajectoryMessageFacotry motionFactory = new SolarPanelWholeBodyTrajectoryMessageFacotry();
   
   public RRTPlannerSolarPanelCleaning rrtPlanner;
   
   private double motionTime;
   
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
   
   public double getMotionTime()
   {      
      return motionTime;
   }
      
   public boolean setWholeBodyTrajectoryMessage(CleaningMotion motion)
   {      
      this.wholeBodyTrajectoryMessage.clear();
      SolarPanelCleaningPose readyPose = new SolarPanelCleaningPose(solarPanel, 0.5, 0.1, -0.2, -Math.PI*0.2);
      
      
           
      switch(motion)
      {
      case ReadyPose:
         PrintTools.info("setTrajectoryMessage -> "+CleaningMotion.ReadyPose);
         this.motionTime = 3.0;
         motionFactory.setMessage(readyPose, Math.PI*0.0, 0.0, this.motionTime);
         wholeBodyTrajectoryMessage = motionFactory.getWholeBodyTrajectoryMessage();
         
         debugPose.add(readyPose.getPose());
         
         break;
         
      case LinearCleaningMotion:
         PrintTools.info("setTrajectoryMessage -> "+CleaningMotion.LinearCleaningMotion);
         
         SolarPanelPath cleaningPath = new SolarPanelPath(readyPose);
         
         cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.1, 0.1, -0.2, -Math.PI*0.2), 4.0);         
         cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.1, 0.2, -0.2, -Math.PI*0.2), 1.0);
         cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.5, 0.2, -0.2, -Math.PI*0.2), 4.0);
         cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.5, 0.3, -0.2, -Math.PI*0.2), 1.0);
         cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.1, 0.3, -0.2, -Math.PI*0.2), 4.0);
                  
         
         this.motionTime = cleaningPath.getArrivalTime().get(cleaningPath.getArrivalTime().size()-1);
         PrintTools.info("motionTime :: "+this.motionTime);
         
         // *** RRT *** //         
         RRTNode1DTimeDomain.cleaningPath = cleaningPath;
         
         RRTNode1DTimeDomain nodeRoot = new RRTNode1DTimeDomain(0.0, 0.0);
         
         rrtPlanner = new RRTPlannerSolarPanelCleaning(nodeRoot, cleaningPath);
         
//         RRTNode1DTimeDomain node1 = new RRTNode1DTimeDomain(0.3, Math.PI*0.3);   
//         RRTNode1DTimeDomain node2 = new RRTNode1DTimeDomain(1.3, Math.PI*0.3);
//         RRTNode1DTimeDomain node3 = new RRTNode1DTimeDomain(2.3, Math.PI*0.3);
//         RRTNode1DTimeDomain node4 = new RRTNode1DTimeDomain(0.3, Math.PI*0.3);
//         RRTNode1DTimeDomain node5 = new RRTNode1DTimeDomain(0.3, Math.PI*0.3);
//         RRTNode1DTimeDomain node6 = new RRTNode1DTimeDomain(2.3, Math.PI*0.3);
//         RRTNode1DTimeDomain node7 = new RRTNode1DTimeDomain(2.3, Math.PI*0.3);
//         RRTNode1DTimeDomain node8 = new RRTNode1DTimeDomain(0.3, Math.PI*0.3);
//         RRTNode1DTimeDomain node9 = new RRTNode1DTimeDomain(2.3, Math.PI*0.3);
//         nodeRoot.isValidNode();
//         PrintTools.info(" "+node1.isValidNode() +" "+node2.isValidNode()+" "+node3.isValidNode()+" "+node4.isValidNode()+" "+node5.isValidNode()+" "+node6.isValidNode()
//         +" "+node7.isValidNode()+" "+node8.isValidNode()+" "+node9.isValidNode());
         
         
         rrtPlanner.expandingTreesAndShortCut(200);
         
        
         PrintTools.info("END shortcutting "+RRTNode1DTimeDomain.nodeValidityTester.cnt);

         // *** message *** //
         PrintTools.info("Putting on Message");
         motionFactory.setCleaningPath(cleaningPath);         
         motionFactory.setMessage(rrtPlanner.getRRTPath());
         
         wholeBodyTrajectoryMessage = motionFactory.getWholeBodyTrajectoryMessage();
         PrintTools.info("Complete putting on message");
         
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
      
      
      
      return true;
   }

   
   
   
   
   
   
}
