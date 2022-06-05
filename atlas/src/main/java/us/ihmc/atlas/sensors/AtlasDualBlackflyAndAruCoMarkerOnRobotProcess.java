package us.ihmc.atlas.sensors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.colorVision.DualBlackflyAndAruCoMarkerOnRobotProcess;
import us.ihmc.avatar.drcRobot.RobotTarget;

public class AtlasDualBlackflyAndAruCoMarkerOnRobotProcess
{
   public AtlasDualBlackflyAndAruCoMarkerOnRobotProcess()
   {
      AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.REAL_ROBOT);
      new DualBlackflyAndAruCoMarkerOnRobotProcess(atlasRobotModel);
   }

   public static void main(String[] args)
   {
      new AtlasDualBlackflyAndAruCoMarkerOnRobotProcess();
   }
}
