package us.ihmc.atlas.communication.producers;

import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBufferTest;

@Disabled
public class AtlasRobotConfigurationDataBufferTest extends RobotConfigurationDataBufferTest
{

   @Override
   public FullHumanoidRobotModel getFullRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.SCS, false).createFullRobotModel();
   }

   @Override
   @Test
   public void testAddingStuff()
   {
      super.testAddingStuff();
   }

   @Override
   @Test
   public void testWaitForTimestamp()
   {
      super.testWaitForTimestamp();
   }
}