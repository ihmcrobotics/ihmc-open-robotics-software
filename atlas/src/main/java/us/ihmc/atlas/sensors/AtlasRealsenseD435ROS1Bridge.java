package us.ihmc.atlas.sensors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasSensorInformation;
import us.ihmc.avatar.sensors.realsense.RealsenseD435ROS1Bridge;

public class AtlasRealsenseD435ROS1Bridge
{
   public AtlasRealsenseD435ROS1Bridge()
   {
      new RealsenseD435ROS1Bridge(new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS),
                                  AtlasSensorInformation.transformPelvisToDepthCamera);
   }

   public static void main(String[] args)
   {
      new AtlasRealsenseD435ROS1Bridge();
   }
}
