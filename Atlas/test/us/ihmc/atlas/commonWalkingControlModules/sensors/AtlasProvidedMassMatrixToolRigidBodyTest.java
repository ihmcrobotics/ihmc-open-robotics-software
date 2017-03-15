package us.ihmc.atlas.commonWalkingControlModules.sensors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasDefaultArmConfigurations;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.sensors.ProvidedMassMatrixToolRigidBodyTest;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.parameters.DefaultArmConfigurations.ArmConfigurations;

public class AtlasProvidedMassMatrixToolRigidBodyTest extends ProvidedMassMatrixToolRigidBodyTest
{
   AtlasRobotVersion version = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ;
   AtlasDefaultArmConfigurations config = new AtlasDefaultArmConfigurations();
   RobotSide side = RobotSide.LEFT;
   AtlasRobotModel atlasRobotModel = new AtlasRobotModel(version, DRCRobotModel.RobotTarget.SCS, false);

   @Override
   public FullHumanoidRobotModel getFullRobotModel()
   {
      FullHumanoidRobotModel fullRobotModel = atlasRobotModel.createFullRobotModel();

      fullRobotModel.setJointAngles(side, LimbName.ARM, config.getArmDefaultConfigurationJointAngles(ArmConfigurations.HOME, side));
      fullRobotModel.updateFrames();

      return fullRobotModel;
   }
}
