package us.ihmc.atlas.kinematicToolboxDiagnostics;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.kinematicsToolboxModule.KinematicToolboxDiagnosticEnvironment;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;

public class AtlasKinematicToolboxDiagnosticEnvironment
{
   public static void main(String[] args)
   {
      AtlasRobotVersion atlasVersion = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ;
      DRCRobotModel.RobotTarget target = DRCRobotModel.RobotTarget.REAL_ROBOT;
      boolean headless = false;
      AtlasRobotModel atlasModel = new AtlasRobotModel(atlasVersion, target, headless);

      new KinematicToolboxDiagnosticEnvironment(atlasModel);
   }
}
