package us.ihmc.valkyrie.obstacleCourse;

import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseFlatTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST, IntegrationCategory.VIDEO})
public class ValkyrieObstacleCourseFlatTest extends DRCObstacleCourseFlatTest
{
   private final DRCRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, false);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
   }

   @Override
   protected Vector3D getFootSlipVector()
   {
      return new Vector3D(0.02, -0.02, 0.0);
   }

   @Override
   protected double getFootSlipTimeDeltaAfterTouchdown()
   {
      return 0.1;
   }

   /**
    * Doesn't work with Valkyrie yet. Need to get it working some day.
    */
   @Ignore
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 40.0)
   @Test(timeout = 300000)
   public void testWalkingUpToRampWithLongStepsAndOccasionallyStraightKnees() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingUpToRampWithLongStepsAndOccasionallyStraightKnees();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 103.8)
   @Test(timeout = 520000)
   public void testSimpleFlatGroundScriptWithOscillatingFeet() throws SimulationExceededMaximumTimeException
   {
      super.testSimpleFlatGroundScriptWithOscillatingFeet();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 64.8)
   @Test(timeout = 320000)
   public void testRotatedStepInTheAir() throws SimulationExceededMaximumTimeException
   {
      super.testRotatedStepInTheAir();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 92.0)
   @Test(timeout = 460000)
   public void testSimpleFlatGroundScriptWithRandomFootSlip() throws SimulationExceededMaximumTimeException
   {
      super.testSimpleFlatGroundScriptWithRandomFootSlip();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 91.2)
   @Test(timeout = 460000)
   public void testWalkingUpToRampWithShortSteps() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingUpToRampWithShortSteps();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 75.5)
   @Test(timeout = 380000)
   public void testSideStepsWithSlipping() throws SimulationExceededMaximumTimeException
   {
      super.testSideStepsWithSlipping();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 30.1)
   @Test(timeout = 150000)
   public void testStandingTooHighToCheckIfSingularityStuffIsWorkingProperly() throws SimulationExceededMaximumTimeException
   {
      super.testStandingTooHighToCheckIfSingularityStuffIsWorkingProperly();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 39.7)
   @Test(timeout = 200000)
   public void testStandingWithOscillatingFeet() throws SimulationExceededMaximumTimeException
   {
      super.testStandingWithOscillatingFeet();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 25.4)
   @Test(timeout = 130000)
   public void testStandingForACoupleSeconds() throws SimulationExceededMaximumTimeException
   {
      super.testStandingForACoupleSeconds();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 78.3)
   @Test(timeout = 390000)
   public void testSideStepsWithRandomSlipping() throws SimulationExceededMaximumTimeException
   {
      super.testSideStepsWithRandomSlipping();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 110.4)
   @Test(timeout = 550000)
   public void testLongStepsMaxHeightPauseAndResume() throws SimulationExceededMaximumTimeException
   {
      super.testLongStepsMaxHeightPauseAndResume();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 92.3, categoriesOverride = {IntegrationCategory.FAST, IntegrationCategory.VIDEO})
   @Test(timeout = 460000)
   public void testTurningInPlaceAndPassingPI() throws SimulationExceededMaximumTimeException
   {
      super.testTurningInPlaceAndPassingPI();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 59.0)
   @Test(timeout = 300000)
   public void testStandingOnUnevenTerrainForACoupleSeconds() throws SimulationExceededMaximumTimeException
   {
      super.testStandingOnUnevenTerrainForACoupleSeconds();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 272.9)
   @Test(timeout = 1400000)
   public void testForMemoryLeaks() throws Exception
   {
      super.testForMemoryLeaks();
   }
}
