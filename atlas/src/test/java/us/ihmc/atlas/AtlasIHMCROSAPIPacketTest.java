package us.ihmc.atlas;

import org.junit.Test;

import us.ihmc.avatar.IHMCROSAPIPacketTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;


@ContinuousIntegrationPlan(categories=IntegrationCategory.IN_DEVELOPMENT)
public class AtlasIHMCROSAPIPacketTest extends IHMCROSAPIPacketTest
{

   @Override
   public DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 8.7)
   @Test(timeout = 420000)
   public void testFuzzyPacketsUsingRos()
   {
      super.testFuzzyPacketsUsingRos();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 2.7)
   @Test(timeout = 420000)
   public void testFuzzyPacketsWithoutRos()
   {
      super.testFuzzyPacketsWithoutRos();
   }

}
