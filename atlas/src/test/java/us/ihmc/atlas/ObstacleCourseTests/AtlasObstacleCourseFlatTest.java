package us.ihmc.atlas.ObstacleCourseTests;

import java.io.IOException;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseFlatTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST, IntegrationCategory.VIDEO})
public class AtlasObstacleCourseFlatTest extends DRCObstacleCourseFlatTest
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
   @ContinuousIntegrationTest(estimatedDuration = 76.7)
   @Test(timeout = 380000)
   public void testACoupleMoreQueuedControllerCommands() throws SimulationExceededMaximumTimeException
   {
      super.testACoupleMoreQueuedControllerCommands();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 78.7)
   @Test(timeout = 390000)
   public void testACoupleQueuedControllerCommands() throws SimulationExceededMaximumTimeException
   {
      super.testACoupleQueuedControllerCommands();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 59.7)
   @Test(timeout = 300000)
   public void testACoupleStepsUsingQueuedControllerCommands() throws SimulationExceededMaximumTimeException
   {
      super.testACoupleStepsUsingQueuedControllerCommands();
   }

   @Override
   // Invoked manually to test memory & thread leaks
   @ContinuousIntegrationTest(estimatedDuration = 50.0, categoriesOverride = IntegrationCategory.MANUAL)
   @Test(timeout = 160000)
   public void testForMemoryLeaks() throws Exception
   {
      super.testForMemoryLeaks();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 113.3)
   @Test(timeout = 570000)
   public void testLongStepsMaxHeightPauseAndResume() throws SimulationExceededMaximumTimeException
   {
      super.testLongStepsMaxHeightPauseAndResume();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 54.2)
   @Test(timeout = 270000)
   public void testRotatedStepInTheAir() throws SimulationExceededMaximumTimeException
   {
      super.testRotatedStepInTheAir();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 84.6)
   @Test(timeout = 420000)
   public void testSideStepsWithRandomSlipping() throws SimulationExceededMaximumTimeException
   {
      super.testSideStepsWithRandomSlipping();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 74.6)
   @Test(timeout = 370000)
   public void testSideStepsWithSlipping() throws SimulationExceededMaximumTimeException
   {
      super.testSideStepsWithSlipping();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 92.6)
   @Test(timeout = 460000)
   public void testSimpleFlatGroundScriptWithOscillatingFeet() throws SimulationExceededMaximumTimeException
   {
      super.testSimpleFlatGroundScriptWithOscillatingFeet();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 82.6)
   @Test(timeout = 410000)
   public void testSimpleFlatGroundScriptWithRandomFootSlip() throws SimulationExceededMaximumTimeException
   {
      super.testSimpleFlatGroundScriptWithRandomFootSlip();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 85.5)
   @Test(timeout = 430000)
   public void testSimpleScripts() throws SimulationExceededMaximumTimeException, IOException
   {
      super.testSimpleScripts();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 26.1)
   @Test(timeout = 130000)
   public void testStandingForACoupleSeconds() throws SimulationExceededMaximumTimeException
   {
      super.testStandingForACoupleSeconds();
   }

   @Override
   // TODO re-enable that test when we have polygon to polygon contact model for SCS
   @ContinuousIntegrationTest(estimatedDuration = 50.0, categoriesOverride = IntegrationCategory.EXCLUDE)
   @Test(timeout = 160000)
   public void testStandingOnUnevenTerrainForACoupleSeconds() throws SimulationExceededMaximumTimeException
   {
      super.testStandingOnUnevenTerrainForACoupleSeconds();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 34.0)
   @Test(timeout = 170000)
   public void testStandingTooHighToCheckIfSingularityStuffIsWorkingProperly() throws SimulationExceededMaximumTimeException
   {
      super.testStandingTooHighToCheckIfSingularityStuffIsWorkingProperly();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 43.2)
   @Test(timeout = 220000)
   public void testStandingWithOscillatingFeet() throws SimulationExceededMaximumTimeException
   {
      super.testStandingWithOscillatingFeet();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 43.2)
   @Test(timeout = 220000)
   public void testStandingWithStateEstimatorDrift() throws SimulationExceededMaximumTimeException
   {
      super.testStandingWithStateEstimatorDrift();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 96.1)
   @Test(timeout = 480000)
   public void testTurningInPlaceAndPassingPI() throws SimulationExceededMaximumTimeException
   {
      super.testTurningInPlaceAndPassingPI();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 68.6)
   @Test(timeout = 340000)
   public void testWalkingUpToRampWithLongStepsAndOccasionallyStraightKnees() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingUpToRampWithLongStepsAndOccasionallyStraightKnees();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 83.3)
   @Test(timeout = 420000)
   public void testWalkingUpToRampWithShortSteps() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingUpToRampWithShortSteps();
   }

   @Override
   protected Vector3D getFootSlipVector()
   {
      return new Vector3D(0.05, -0.07, 0.0);//(0.06, -0.06, 0.0);
   }

   @Override
   protected double getFootSlipTimeDeltaAfterTouchdown()
   {
      return 0.0;
   }
}
