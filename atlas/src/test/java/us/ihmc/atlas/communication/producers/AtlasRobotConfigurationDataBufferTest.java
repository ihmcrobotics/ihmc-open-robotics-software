package us.ihmc.atlas.communication.producers;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBufferTest;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FLAKY)
public class AtlasRobotConfigurationDataBufferTest extends RobotConfigurationDataBufferTest
{

   @Override
   public FullHumanoidRobotModel getFullRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.SCS, false).createFullRobotModel();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.5)
   @Test(timeout = 30000)
   public void testAddingStuff()
   {
      super.testAddingStuff();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 100.0)
   @Test(timeout = 30000)
   public void testWaitForTimestamp()
   {
      super.testWaitForTimestamp();
   }
}