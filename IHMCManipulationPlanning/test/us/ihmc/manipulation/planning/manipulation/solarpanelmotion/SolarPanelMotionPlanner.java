package us.ihmc.manipulation.planning.manipulation.solarpanelmotion;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.manipulation.planning.rrttimedomain.RRTNode1DTimeDomain;
import us.ihmc.manipulation.planning.rrttimedomain.RRTPlannerSolarPanelCleaning;
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
         
         cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.1, 0.1, -0.1, -Math.PI*0.2), 5.0);         
         cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.1, 0.2, -0.1, -Math.PI*0.2), 1.0);
         cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.5, 0.2, -0.1, -Math.PI*0.2), 5.0);
         cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.5, 0.3, -0.1, -Math.PI*0.2), 1.0);
         cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.1, 0.3, -0.1, -Math.PI*0.2), 5.0);
                  
         
         this.motionTime = cleaningPath.getArrivalTime().get(cleaningPath.getArrivalTime().size()-1);
         PrintTools.info("motionTime :: "+this.motionTime);
         
         // *** RRT *** //         
         RRTNode1DTimeDomain.cleaningPath = cleaningPath;
         RRTNode1DTimeDomain nodeRoot = new RRTNode1DTimeDomain(0.0, 0.0);
         rrtPlanner = new RRTPlannerSolarPanelCleaning(nodeRoot, cleaningPath);
         
         rrtPlanner.expandingTreesAndShortCut(200);
         
         for(int i=0;i<rrtPlanner.getNumberOfPlanners();i++)
         {
            PrintTools.info("path "+i);
            for(int k=0;k<rrtPlanner.getPlanner(i).getOptimalPath().size();k++)
            {
               PrintTools.info("anOptimalPath "+rrtPlanner.getPlanner(i).getOptimalPath().get(k).getNodeData(0));
            }   
         }
               
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
