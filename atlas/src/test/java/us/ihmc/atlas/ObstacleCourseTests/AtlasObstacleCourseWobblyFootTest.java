package us.ihmc.atlas.ObstacleCourseTests;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseWobblyFootTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.wholeBodyController.FootContactPoints;
import us.ihmc.wholeBodyController.WobblySimulationContactPoints;
import us.ihmc.yoVariables.variable.YoDouble;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.SLOW, IntegrationCategory.VIDEO})
public class AtlasObstacleCourseWobblyFootTest extends DRCObstacleCourseWobblyFootTest
{
   private static final double footZWobbleForTests = 0.01;

   @Override
   public DRCRobotModel getRobotModel()
   {
      final AtlasRobotVersion atlasVersion = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;
      FootContactPoints simulationContactPoints = new WobblySimulationContactPoints(footZWobbleForTests);
      return new AtlasRobotModel(atlasVersion, RobotTarget.SCS, false, simulationContactPoints);
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.5)
   @Test(timeout = 100000)
   public void testStandingForACoupleSecondsWithWobblyFeet() throws SimulationExceededMaximumTimeException
   {
      super.testStandingForACoupleSecondsWithWobblyFeet();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 47.9, categoriesOverride = {IntegrationCategory.FLAKY, IntegrationCategory.VIDEO})
   @Test(timeout = 100000)
   public void testTurningInPlaceAndPassingPIWithWobblyFeet() throws SimulationExceededMaximumTimeException
   {
      super.testTurningInPlaceAndPassingPIWithWobblyFeet();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 44.4, categoriesOverride = {IntegrationCategory.FLAKY, IntegrationCategory.VIDEO})
   @Test(timeout = 100000)
   public void testWalkingUpToRampWithShortStepsWithWobblyFeet() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingUpToRampWithShortStepsWithWobblyFeet();
   }

   @Override
   protected YoDouble getPelvisOrientationErrorVariableName(SimulationConstructionSet scs)
   {
      return (YoDouble) scs.getVariable("root.atlas.DRCSimulation.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WholeBodyControllerCore.WholeBodyFeedbackController.pelvisOrientationFBController.pelvisAxisAngleOrientationController",
            "pelvisRotationErrorInBodyZ");
   }
}
