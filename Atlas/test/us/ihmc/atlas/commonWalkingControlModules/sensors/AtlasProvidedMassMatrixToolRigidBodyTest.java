package us.ihmc.atlas.commonWalkingControlModules.sensors;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.SdfLoader.partNames.LimbName;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasArmControllerParameters;
import us.ihmc.atlas.parameters.AtlasDefaultArmConfigurations;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.sensors.ProvidedMassMatrixToolRigidBodyTest;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.parameters.DefaultArmConfigurations.ArmConfigurations;

public class AtlasProvidedMassMatrixToolRigidBodyTest extends ProvidedMassMatrixToolRigidBodyTest
{
   AtlasRobotVersion version = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ;
   AtlasDefaultArmConfigurations config = new AtlasDefaultArmConfigurations();
   RobotSide side = RobotSide.LEFT;
   
   @Override
   public FullHumanoidRobotModel getFullRobotModel()
   {
      SDFFullHumanoidRobotModel fullRobotModel = new AtlasRobotModel(version, AtlasRobotModel.AtlasTarget.SIM, false).createFullRobotModel();
      
      fullRobotModel.setJointAngles(side, LimbName.ARM, config.getArmDefaultConfigurationJointAngles(ArmConfigurations.HOME, side));
      fullRobotModel.updateFrames();
      
      return fullRobotModel;
   }
   
   @Override
   public ArmControllerParameters getArmControllerParameters()
   {
      return new AtlasArmControllerParameters(false, version.getDistanceAttachmentPlateHand());
   }
}
