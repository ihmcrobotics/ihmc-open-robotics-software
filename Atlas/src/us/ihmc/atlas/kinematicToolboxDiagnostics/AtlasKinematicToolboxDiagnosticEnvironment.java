package us.ihmc.atlas.kinematicToolboxDiagnostics;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicToolboxDiagnosticEnvironment;

public class AtlasKinematicToolboxDiagnosticEnvironment
{
   public static void main(String[] args)
   {
      AtlasRobotVersion atlasVersion = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ;
      RobotTarget target = RobotTarget.REAL_ROBOT;
      boolean headless = false;
      AtlasRobotModel atlasModel = new AtlasRobotModel(atlasVersion, target, headless);

      new KinematicToolboxDiagnosticEnvironment(atlasModel);
   }
}
