package us.ihmc.communication.packets.sensing;

import us.ihmc.communication.packets.Packet;

public class DepthDataClearCommand extends Packet<DepthDataClearCommand>
{
   public enum DepthDataTree
   {
      DECAY_POINT_CLOUD,
      QUADTREE
   }
   
   public DepthDataTree depthDataTree;
   
   public DepthDataClearCommand()
   {
      
   }
   
   public DepthDataClearCommand(DepthDataTree depthDataTree)
   {
      this.depthDataTree = depthDataTree;
   }
   
   public DepthDataTree getDepthDataTree()
   {
      return depthDataTree;
   }
   

   @Override
   public boolean equals(Object other)
   {
      if(other instanceof DepthDataClearCommand)
      {
         return ((DepthDataClearCommand) other).depthDataTree == depthDataTree;
      }
      else
      {
         return false;
      }
   }

   @Override
   public boolean epsilonEquals(DepthDataClearCommand other, double epsilon)
   {
      return equals(other);
   }
}
