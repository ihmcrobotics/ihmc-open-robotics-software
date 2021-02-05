package us.ihmc.atlas.sensors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasSensorInformation;
import us.ihmc.avatar.sensors.realsense.RealsenseROS1Bridge;

public class AtlasRealsenseROS1Bridge
{
   public AtlasRealsenseROS1Bridge()
   {
      new RealsenseROS1Bridge(new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS),
                              AtlasSensorInformation.transformPelvisToD435DepthCamera,
                              AtlasSensorInformation.transformPelvisToL515DepthCamera);
   }

   public static void main(String[] args)
   {
      new AtlasRealsenseROS1Bridge();
   }
}
