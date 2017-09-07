package us.ihmc.humanoidBehaviors.behaviors.solarPanel;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;
import us.ihmc.manipulation.planning.rrt.RRTNode;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanelLinearPath;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanelPath;

public class RRTPlannerSolarPanelCleaning
{
   private RRTNode rootNode;
   private SolarPanelPath cleaningPath;
   
   public ArrayList<RRTPlannerTimeDomain> sequentialPlanner = new ArrayList<RRTPlannerTimeDomain>();
   private ArrayList<RRTNode> rrtPath = new ArrayList<RRTNode>();
   
   public RRTPlannerSolarPanelCleaning(RRTNode rootNode, SolarPanelPath cleaningPath)
   {
      this.rootNode = rootNode;
      this.cleaningPath = cleaningPath;

      ArrayList<SolarPanelLinearPath> linearPath = this.cleaningPath.getLinearPath();
      for(int i=0;i<linearPath.size();i++)
      {           
         RRTNode1DTimeDomain rootNodeOfPlanner = new RRTNode1DTimeDomain();
         RRTPlannerTimeDomain aPlanner = new RRTPlannerTimeDomain(rootNodeOfPlanner);
         
         aPlanner.getTree().getRootNode().setNodeData(rootNode);
         
         double treeMaximumTime = linearPath.get(i).getMotionEndTime();
         double treeTimeInterval = linearPath.get(i).getMotionEndTime() - linearPath.get(i).getMotionStartTime();
         
         RRTNode1DTimeDomain nodeLowerBound = new RRTNode1DTimeDomain(linearPath.get(i).getMotionStartTime(), -Math.PI*0.2);
         RRTNode1DTimeDomain nodeUpperBound = new RRTNode1DTimeDomain(linearPath.get(i).getMotionStartTime()+treeTimeInterval*1.5, Math.PI*0.2);
         
         aPlanner.getTree().setMaximumMotionTime(treeMaximumTime);
         aPlanner.getTree().setUpperBound(nodeUpperBound);
         aPlanner.getTree().setLowerBound(nodeLowerBound);
         sequentialPlanner.add(aPlanner);
      }   
   }
   
   public int getNumberOfPlanners()
   {
      return sequentialPlanner.size();
   }
   
   public RRTPlannerTimeDomain getPlanner(int index)
   {
      return sequentialPlanner.get(index);
   }
   
   public void expandingTreesAndShortCut(int maxNumberOfExpanding)
   {
      for(int i=0;i<getNumberOfPlanners();i++)
      {
         PrintTools.info("Start Expanding "+i+" tree");
         getPlanner(i).expandTreeGoal(maxNumberOfExpanding);
         
         getPlanner(i).updateOptimalPath(30);
         
         if(i < getNumberOfPlanners()-1)
         {
            ArrayList<RRTNode> optimalPath = getPlanner(i).getOptimalPath();
            RRTNode newRootNode = optimalPath.get(optimalPath.size()-1);
            
            getPlanner(i+1).getTree().getRootNode().setNodeData(newRootNode);
         }
         PrintTools.info("Finish Expanding "+i+" tree");
         
      }
      
      PrintTools.info("getNumberOfPlanners "+getNumberOfPlanners());
      for(int i=0;i<getNumberOfPlanners();i++)
      {
         if(i==0)
         {
            for(int j=0;j<getPlanner(i).getOptimalPath().size();j++)
            {
               rrtPath.add(getPlanner(i).getOptimalPath().get(j));
            }   
         }
         else
         {
            for(int j=1;j<getPlanner(i).getOptimalPath().size();j++)
            {
               rrtPath.add(getPlanner(i).getOptimalPath().get(j));
            }
         }
      }
      
   }
   
   public ArrayList<RRTNode> getRRTPath()
   {
      PrintTools.info(""+rrtPath.size());
      return rrtPath;
   }
   
}
