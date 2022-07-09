package us.ihmc.atlas.sensors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.lidar.OusterLidarOnRobotProcess;

public class AtlasOusterLidarOnRobotProcess
{
   public AtlasOusterLidarOnRobotProcess()
   {
      AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.REAL_ROBOT);
      new OusterLidarOnRobotProcess(atlasRobotModel);
   }

   public static void main(String[] args)
   {
      new AtlasOusterLidarOnRobotProcess();
   }
}
