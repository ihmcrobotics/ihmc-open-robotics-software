package us.ihmc.humanoidBehaviors.behaviors.rrtPlanner;

import us.ihmc.manipulation.planning.rrt.RRTNode;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanelPath;

public class TimeDomain1DNode extends RRTNode
{
   private boolean isValidNode = false;
   public static SolarPanelPath cleaningPath;
   
   public TimeDomain1DNode()
   {
      super(2);
   }
   
   public TimeDomain1DNode(double time, double pelvisYaw)
   {
      super(2);
      super.setNodeData(0, time);
      super.setNodeData(1, pelvisYaw);
   }
   
   public void setIsValidNode(boolean setValue)
   {
      isValidNode = setValue;
   }
   
   @Override
   public boolean isValidNode()
   {      
      return isValidNode;
   }

   @Override
   public RRTNode createNode()
   {
      return new TimeDomain1DNode();
   }

}
