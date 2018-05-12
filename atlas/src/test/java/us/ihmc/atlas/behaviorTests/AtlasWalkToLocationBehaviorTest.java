package us.ihmc.atlas.behaviorTests;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.behaviorTests.DRCWalkToLocationBehaviorTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class AtlasWalkToLocationBehaviorTest extends DRCWalkToLocationBehaviorTest
{
   private final AtlasRobotModel robotModel;

   public AtlasWalkToLocationBehaviorTest()
   {
      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
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
   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 150000)
   public void testTurn361DegreesInPlace() throws SimulationExceededMaximumTimeException
   {
      super.testTurn361DegreesInPlace();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 57.2)
   @Test(timeout = 290000)
   public void testWalkAndStopBehavior() throws SimulationExceededMaximumTimeException
   {
      super.testWalkAndStopBehavior();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 60.3)
   @Test(timeout = 300000)
   public void testWalkAtAngleAndFinishAlignedWithInitialOrientation() throws SimulationExceededMaximumTimeException
   {
      super.testWalkAtAngleAndFinishAlignedWithInitialOrientation();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 63.3)
   @Test(timeout = 320000)
   public void testWalkAtAngleAndFinishAlignedWithWalkingPath() throws SimulationExceededMaximumTimeException
   {
      super.testWalkAtAngleAndFinishAlignedWithWalkingPath();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 71.6)
   @Test(timeout = 360000)
   public void testWalkAtAngleUsingStartOrientation() throws SimulationExceededMaximumTimeException
   {
      super.testWalkAtAngleUsingStartOrientation();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 84.3)
   @Test(timeout = 420000)
   public void testWalkAtAngleUsingStartTargetMeanOrientation() throws SimulationExceededMaximumTimeException
   {
      super.testWalkAtAngleUsingStartTargetMeanOrientation();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 108.6)
   @Test(timeout = 540000)
   public void testWalkAtAngleUsingTargetOrientation() throws SimulationExceededMaximumTimeException
   {
      super.testWalkAtAngleUsingTargetOrientation();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 44.0)
   @Test(timeout = 220000)
   public void testWalkBackwardsASmallAmountWithoutTurningInPlace() throws SimulationExceededMaximumTimeException
   {
      super.testWalkBackwardsASmallAmountWithoutTurningInPlace();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 59.4)
   @Test(timeout = 300000)
   public void testWalkForwardsX() throws SimulationExceededMaximumTimeException
   {
      super.testWalkForwardsX();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 57.8, categoriesOverride = {IntegrationCategory.EXCLUDE})
   @Test(timeout = 290000)
   public void testWalkPauseAndResumeBehavior() throws SimulationExceededMaximumTimeException
   {
      super.testWalkPauseAndResumeBehavior();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 45.7, categoriesOverride = {IntegrationCategory.EXCLUDE})
   @Test(timeout = 230000)
   public void testWalkPauseAndResumeOnLastStepBehavior() throws SimulationExceededMaximumTimeException
   {
      super.testWalkPauseAndResumeOnLastStepBehavior();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 67.6, categoriesOverride = {IntegrationCategory.EXCLUDE})
   @Test(timeout = 340000)
   public void testWalkStopAndWalkToDifferentLocation() throws SimulationExceededMaximumTimeException
   {
      super.testWalkStopAndWalkToDifferentLocation();
   }
}
