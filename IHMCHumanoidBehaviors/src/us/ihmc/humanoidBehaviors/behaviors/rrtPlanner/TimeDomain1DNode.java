package us.ihmc.humanoidBehaviors.behaviors.rrtPlanner;

import java.util.Random;

import us.ihmc.manipulation.planning.rrt.RRTNode;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanelPath;

public class TimeDomain1DNode extends RRTNode
{
   private boolean isValidNode = false;
   
   public static double upperBound1D = Math.PI*0.2;
   public static double lowerBound1D = -Math.PI*0.2;
   
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
   public void setRandomNodeData()
   {
      Random randomManager = new Random();

      double randonValue = randomManager.nextDouble() * (upperBound1D - lowerBound1D) + lowerBound1D;
      setNodeData(1, randonValue);
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
