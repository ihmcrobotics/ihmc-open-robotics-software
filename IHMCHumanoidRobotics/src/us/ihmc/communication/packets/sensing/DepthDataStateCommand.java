package us.ihmc.communication.packets.sensing;

import us.ihmc.communication.packets.Packet;

public class DepthDataStateCommand extends Packet<DepthDataStateCommand>
{
   public enum LidarState
   {
      DISABLE, ENABLE
   }
   
   public LidarState lidarState;
   
   public DepthDataStateCommand()
   {
   }

   public DepthDataStateCommand(LidarState lidarState)
   {
      this.lidarState = lidarState;
   }

   public LidarState getLidarState()
   {
      return lidarState;
   }
   
   @Override
   public boolean equals(Object other)
   {
      if(other instanceof DepthDataStateCommand)
      {
         return ((DepthDataStateCommand) other).lidarState == lidarState;
      }
      else
      {
         return false;
      }
   }

   @Override
   public boolean epsilonEquals(DepthDataStateCommand other, double epsilon)
   {
      return equals(other);
   }
}