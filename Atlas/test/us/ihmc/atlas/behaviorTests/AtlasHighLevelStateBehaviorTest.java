package us.ihmc.atlas.behaviorTests;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.behaviorTests.DRCHighLevelStateBehaviorTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class AtlasHighLevelStateBehaviorTest extends DRCHighLevelStateBehaviorTest
{
   private final AtlasRobotModel robotModel;

   public AtlasHighLevelStateBehaviorTest()
   {
      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, DRCRobotModel.RobotTarget.SCS, false);
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
   @ContinuousIntegrationTest(estimatedDuration = 26.3)
   @Test(timeout = 130000)
   public void testWalkingState() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingState();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 33.0)
   @Test(timeout = 160000)
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
