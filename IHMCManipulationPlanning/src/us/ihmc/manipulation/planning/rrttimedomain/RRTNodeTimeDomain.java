package us.ihmc.manipulation.planning.rrttimedomain;

import us.ihmc.manipulation.planning.manipulation.solarpanelmotion.SolarPanelPoseValidityTester;
import us.ihmc.manipulation.planning.rrt.RRTNode;

public abstract class RRTNodeTimeDomain extends RRTNode
{
   public static SolarPanelPoseValidityTester nodeValidityTester;
   
   /*
    * getNodeData(0) = time
    */
    
   
   
   public double getTime()
   {
      return getNodeData(0);
   }
   
   public double getDisplacement(RRTNodeTimeDomain parentNode)
   {
      double squaredDisplacement = 0;
      
      for(int i=1;i<getDimensionOfNodeData();i++)
      {
         squaredDisplacement = squaredDisplacement + (parentNode.getNodeData(i)-getNodeData(i))*(parentNode.getNodeData(i)-getNodeData(i));
      }
      
      double displacement = Math.sqrt(squaredDisplacement);
      
      return displacement;
   }
   
   public double getSpeed(RRTNodeTimeDomain parentNode)
   {
      if(getTime() - parentNode.getTime() > 0)
      {
         return getDisplacement(parentNode)/(getTime()-parentNode.getTime());         
      }
      else
      {  
         return 0;
      }
   }
}
