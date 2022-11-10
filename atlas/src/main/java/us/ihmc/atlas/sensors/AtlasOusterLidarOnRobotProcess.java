package us.ihmc.atlas.sensors;

import us.ihmc.avatar.lidar.OusterDepthPublisher;

public class AtlasOusterLidarOnRobotProcess
{
   public AtlasOusterLidarOnRobotProcess()
   {
      new OusterDepthPublisher();
   }

   public static void main(String[] args)
   {
      new AtlasOusterLidarOnRobotProcess();
   }
}
