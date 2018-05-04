package us.ihmc.atlas.behaviorTests;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.behaviorTests.DRCHighLevelStateBehaviorTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class AtlasHighLevelStateBehaviorTest extends DRCHighLevelStateBehaviorTest
{
   private final AtlasRobotModel robotModel;

   public AtlasHighLevelStateBehaviorTest()
   {
      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.SCS, false);
   }

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
   @ContinuousIntegrationTest(estimatedDuration = 44.9)
   @Test(timeout = 220000)
   public void testWalkingState() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingState();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 43.9)
   @Test(timeout = 220000)
   public void testDoNothingBahviourState() throws SimulationExceededMaximumTimeException
   {
      super.testDoNothingBahviourState();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
   @Test(timeout = 300000)
   public void testDiagnosticsState() throws SimulationExceededMaximumTimeException
   {
      super.testDiagnosticsState();
   }
}
