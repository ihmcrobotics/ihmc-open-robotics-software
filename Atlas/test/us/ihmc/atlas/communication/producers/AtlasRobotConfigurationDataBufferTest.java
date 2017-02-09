package us.ihmc.atlas.communication.producers;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBufferTest;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FLAKY)
public class AtlasRobotConfigurationDataBufferTest extends RobotConfigurationDataBufferTest
{

   @Override
   public FullHumanoidRobotModel getFullRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, DRCRobotModel.RobotTarget.SCS, false).createFullRobotModel();
   }

}