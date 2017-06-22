package us.ihmc.manipulation.planning.rrt.wholebodyplanning;

public class TaskNodeRegion
{
   private NodeData upperLimit;
   private NodeData lowerLimit;
   
   public TaskNodeRegion(int size)
   {
      upperLimit = new NodeData(size);
      lowerLimit = new NodeData(size);
   }
   
   public void setRandomRegion(int index, double lowerValue, double upperValue)
   {
      upperLimit.setQ(index, upperValue);
      lowerLimit.setQ(index, lowerValue);
   }
   
   public NodeData getUpperLimit()
   {
      return upperLimit;
   }
   
   public NodeData getLowerLimit()
   {
      return lowerLimit;
   }
}
