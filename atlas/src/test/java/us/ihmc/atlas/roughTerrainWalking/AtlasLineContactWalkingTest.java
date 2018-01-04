package us.ihmc.atlas.roughTerrainWalking;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.HumanoidLineContactWalkingTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.IN_DEVELOPMENT})
public class AtlasLineContactWalkingTest extends HumanoidLineContactWalkingTest
{
   private final DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 100.0, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
   @Test(timeout = 30000)
   public void testWalkingOnLines() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingOnLines();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 100.0, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
   @Test(timeout = 30000)
   public void testWalkingOnStraightForwardLines() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingOnStraightForwardLines();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 100.0, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
   @Test(timeout = 30000)
   public void testWalkingOnStraightSidewayLines() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingOnStraightSidewayLines();
   }
}
