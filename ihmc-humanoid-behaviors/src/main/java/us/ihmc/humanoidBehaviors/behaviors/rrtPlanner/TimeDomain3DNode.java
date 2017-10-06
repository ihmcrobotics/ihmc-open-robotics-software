package us.ihmc.humanoidBehaviors.behaviors.rrtPlanner;

import java.util.Random;

import us.ihmc.commons.PrintTools;
import us.ihmc.manipulation.planning.rrt.RRTNode;

public class TimeDomain3DNode extends RRTNode
{
   private boolean isValidNode = false;
   
   // pelvisHeight
   public static double defaultPelvisHeight;
   public static double upperShiftedBound1D = 0.10;
   public static double lowerShiftedBound1D = -0.100;
   
   // chestYaw
   public static double upperBound2D = Math.PI*0.2;
   public static double lowerBound2D = -Math.PI*0.2;
   
   // chestPitch
   public static double upperBound3D = Math.PI*0.100;
   public static double lowerBound3D = -Math.PI*0.100;
   
   
   
   public TimeDomain3DNode()
   {
      super(4);
      super.setNodeData(1, defaultPelvisHeight);
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
   
   @Override
   public void setRandomNodeData()
   {
      Random randomManager = new Random();

      double upperBound1D = defaultPelvisHeight + upperShiftedBound1D;
      double lowerBound1D = defaultPelvisHeight - lowerShiftedBound1D;
      
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
