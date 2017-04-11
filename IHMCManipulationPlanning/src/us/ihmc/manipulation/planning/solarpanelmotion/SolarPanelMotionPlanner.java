package us.ihmc.manipulation.planning.solarpanelmotion;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.manipulation.planning.rrttimedomain.RRTNode1DTimeDomain;
import us.ihmc.manipulation.planning.rrttimedomain.RRTPlanner1DTimeDomain;
import us.ihmc.robotics.geometry.transformables.Pose;

public class SolarPanelMotionPlanner
{
   private SolarPanel solarPanel;
   
   WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();   
   SolarPanelWholeBodyTrajectoryMessageFacotry motionFactory = new SolarPanelWholeBodyTrajectoryMessageFacotry();
   
   public RRTPlanner1DTimeDomain plannerTimeDomain;
   
   private double motionTime;
   
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
   
   public double getMotionTime()
   {      
      return motionTime;
   }
      
   public boolean setWholeBodyTrajectoryMessage(CleaningMotion motion)
   {      
      this.wholeBodyTrajectoryMessage.clear();
      SolarPanelCleaningPose readyPose = new SolarPanelCleaningPose(solarPanel, 0.5, 0.1, -0.1, -Math.PI*0.2);
      
      
           
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
                  
         SolarPanelCleaningPose endPose = new SolarPanelCleaningPose(solarPanel, 0.1, 0.1, -0.1, -Math.PI*0.2);//
         cleaningPath.addCleaningPose(endPose, 5.0);
         cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.1, 0.2, -0.1, -Math.PI*0.2), 1.0);
         cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.5, 0.2, -0.1, -Math.PI*0.2), 5.0);
//         cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.5, 0.3, -0.1, -Math.PI*0.2), 1.0);
//         
//         cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.1, 0.3, -0.1, -Math.PI*0.2), 5.0);
//         cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.1, 0.4, -0.1, -Math.PI*0.2), 1.0);
//         cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.5, 0.4, -0.1, -Math.PI*0.2), 5.0);
//         cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.5, 0.5, -0.1, -Math.PI*0.2), 1.0);
         
         
         this.motionTime = cleaningPath.getArrivalTime().get(cleaningPath.getArrivalTime().size()-1);
         PrintTools.info("motionTime :: "+this.motionTime);
         
         // *** RRT *** //         
         /*
         
         RRTNode1DTimeDomain nodeLowerBound = new RRTNode1DTimeDomain(0.0, -Math.PI*0.4);
         RRTNode1DTimeDomain nodeUpperBound = new RRTNode1DTimeDomain(motionTime*1.5, Math.PI*0.4);
         
         
         plannerTimeDomain.getTree().setMotionTime(motionTime);
         plannerTimeDomain.getTree().setUpperBound(nodeUpperBound);
         plannerTimeDomain.getTree().setLowerBound(nodeLowerBound);
         
         PrintTools.info("END setting");
         RRTNode1DTimeDomain.cleaningPath = cleaningPath;         
                  
         plannerTimeDomain.expandTreeGoal(200);
         PrintTools.info("END expanding");
         plannerTimeDomain.updateOptimalPath(30);
         PrintTools.info("END shortcutting "+RRTNode1DTimeDomain.nodeValidityTester.cnt);

         // *** message *** //
         PrintTools.info("Putting on Message");
         motionFactory.setCleaningPath(cleaningPath);
         */
         
         //motionFactory.setMessage(plannerTimeDomain.getOptimalPath()); -----
         
         
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
