package us.ihmc.manipulation.planning.rrt.wholebodyplanning;

public class TaskNode3D extends TaskNode
{   
   private boolean isValidNode;  
   
   
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
   

   

   
   public void setIsValidNode(boolean value)
   {
      isValidNode = value;
   }
   
   @Override
   public boolean isValidNode()
   {      
      return isValidNode;
   }
}
