package us.ihmc.humanoidBehaviors.behaviors.rrtPlanner;

import java.util.Random;

import us.ihmc.manipulation.planning.rrt.RRTNode;

public class TimeDomain4DNode extends RRTNode
{
private boolean isValidNode = false;
   
   // handYaw
   public static double upperBound1D = 0.06;
   public static double lowerBound1D = -0.100;

   // pelvisHeight
   public static double defaultPelvisHeight;
   public static double upperShiftedBound1D = 0.06;
   public static double lowerShiftedBound1D = -0.100;
   
   // chestYaw
   public static double upperBound2D = Math.PI*0.2;
   public static double lowerBound2D = -Math.PI*0.0;
   
   // chestPitch
   public static double upperBound3D = Math.PI*0.15;
   public static double lowerBound3D = -Math.PI*0.05;
   
   
   
   public TimeDomain4DNode()
   {
      super(5);
      super.setNodeData(1, defaultPelvisHeight);
   }
   
   public TimeDomain4DNode(double time, double handYaw, double pelvisHeight, double chestYaw, double chestPitch)
   {
      super(5);
      super.setNodeData(0, time);
      super.setNodeData(1, handYaw);
      super.setNodeData(2, pelvisHeight);
      super.setNodeData(3, chestYaw);
      super.setNodeData(4, chestPitch);
   }
   
   public void setIsValidNode(boolean setValue)
   {
      isValidNode = setValue;
   }
   
   @Override
   public void setRandomNodeData()
   {
//      Random randomManager = new Random();
//
//      double upperBound1D = defaultPelvisHeight + upperShiftedBound1D;
//      double lowerBound1D = defaultPelvisHeight - lowerShiftedBound1D;
//      
//      double randonValue;
//      randonValue = randomManager.nextDouble() * (upperBound1D - lowerBound1D) + lowerBound1D;
//      setNodeData(1, randonValue);
//           
//      randonValue = randomManager.nextDouble() * (upperBound2D - lowerBound2D) + lowerBound2D;
//      setNodeData(2, randonValue);
//      
//      randonValue = randomManager.nextDouble() * (upperBound3D - lowerBound3D) + lowerBound3D;
//      setNodeData(3, randonValue);  
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
