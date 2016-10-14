package us.ihmc.humanoidRobotics.communication.packets.sensing;

import us.ihmc.communication.packets.Packet;

public class DepthDataStateCommand extends Packet<DepthDataStateCommand>
{
   public enum LidarState
   {
      DISABLE, ENABLE, ENABLE_BEHAVIOR_ONLY
   }

   public LidarState lidarState;

   public boolean publishLidarPose = false;

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

   public boolean isLidarPoseRequested()
   {
      return publishLidarPose;
   }

   @Override
   public boolean equals(Object other)
   {
      if (other instanceof DepthDataStateCommand)
      {
         DepthDataStateCommand depthDataStateCommand = (DepthDataStateCommand) other;
         return depthDataStateCommand.lidarState == lidarState && depthDataStateCommand.publishLidarPose == publishLidarPose;
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