package us.ihmc.humanoidBehaviors.behaviors.rrtPlanner;

import java.util.Random;

import us.ihmc.manipulation.planning.rrt.RRTNode;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanelPath;

public class TimeDomain3DNode extends RRTNode
{
   private boolean isValidNode = false;
   public static SolarPanelPath cleaningPath;
   
   // pelvisHeight
   public static double upperBound1D = Math.PI*0.2;
   public static double lowerBound1D = -Math.PI*0.2;
   
   // chestYaw
   public static double upperBound2D = Math.PI*0.2;
   public static double lowerBound2D = -Math.PI*0.2;
   
   // chestPitch
   public static double upperBound3D = Math.PI*0.2;
   public static double lowerBound3D = -Math.PI*0.2;
   
   public TimeDomain3DNode()
   {
      super(4);
   }
   
   public TimeDomain3DNode(double time, double pelvisHeight, double chestYaw, double chestPitch)
   {
      super(4);
      super.setNodeData(0, time);
      super.setNodeData(1, pelvisHeight);
      super.setNodeData(2, chestYaw);
      super.setNodeData(3, chestPitch);
   }
   
   public void setIsValidNode(boolean setValue)
   {
      isValidNode = setValue;
   }
   
   public void setRandomNodeData()
   {
      Random randomManager = new Random();

      double randonValue;
      randonValue = randomManager.nextDouble() * (upperBound1D - lowerBound1D) + lowerBound1D;
      setNodeData(1, randonValue);
      
      randonValue = randomManager.nextDouble() * (upperBound2D - lowerBound2D) + lowerBound2D;
      setNodeData(2, randonValue);
      
      randonValue = randomManager.nextDouble() * (upperBound3D - lowerBound3D) + lowerBound3D;
      setNodeData(3, randonValue);  
   }
   
   @Override
   public boolean isValidNode()
   {      
      return isValidNode;
   }

   @Override
   public RRTNode createNode()
   {
      return new TimeDomain3DNode();
   }

}
