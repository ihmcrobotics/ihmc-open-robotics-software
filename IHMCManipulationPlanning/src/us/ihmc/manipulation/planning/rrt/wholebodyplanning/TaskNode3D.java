package us.ihmc.manipulation.planning.rrt.wholebodyplanning;

import us.ihmc.commons.PrintTools;

public class TaskNode3D extends TaskNode
{   
   public static WheneverWholeBodyKinematicsSolver nodeTester;
   
   
   public TaskNode3D()
   {
      super(4);      
   }
   
   public TaskNode3D(double time, double pelvisHeight, double chestYaw, double chestPitch)
   {
      super(4);
      setNodeData(0, time);
      setNodeData(1, pelvisHeight);
      setNodeData(2, chestYaw);
      setNodeData(3, chestPitch);
   }
   
   @Override
   public boolean isValidNode()
   {
      /*
       * using whenever 
       */
      
      
      double scaledDistance = 0;
      scaledDistance = scaledDistance + (getNodeData(0) - 5.0)*(getNodeData(0) - 5.0);
      scaledDistance = scaledDistance + (getNodeData(1) - 0.0)*(getNodeData(1) - 0.0);
      scaledDistance = scaledDistance + (getNodeData(2) - 0.0)*(getNodeData(2) - 0.0);
      scaledDistance = scaledDistance + (getNodeData(3) - 0.0)*(getNodeData(3) - 0.0);
      
      scaledDistance = Math.sqrt(scaledDistance);
      
      if(scaledDistance < 0.4)
      {
         setIsValidNode(false);
      }
      else
      {
         setIsValidNode(true);
      }
      
      
      
      
      
      return isValid;
   }

   @Override
   public TaskNode createNode()
   {
      return new TaskNode3D();
   }

   

   
}
