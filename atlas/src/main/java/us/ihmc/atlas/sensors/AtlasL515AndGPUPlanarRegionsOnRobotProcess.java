package us.ihmc.atlas.sensors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.gpuPlanarRegions.L515AndGPUPlanarRegionsOnRobotProcess;

public class AtlasL515AndGPUPlanarRegionsOnRobotProcess
{
   public AtlasL515AndGPUPlanarRegionsOnRobotProcess()
   {
      AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.REAL_ROBOT);
      new L515AndGPUPlanarRegionsOnRobotProcess(atlasRobotModel);
   }

   public static void main(String[] args)
   {
      new AtlasL515AndGPUPlanarRegionsOnRobotProcess();
   }
}
