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
      
      
      
      
      
      
      
      
      
      
      
      
      return isValid;
   }

   @Override
   public TaskNode createNode()
   {
      return new TaskNode3D();
   }

   

   
}
