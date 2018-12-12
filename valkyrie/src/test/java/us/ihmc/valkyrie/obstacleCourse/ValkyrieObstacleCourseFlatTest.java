package us.ihmc.valkyrie.obstacleCourse;

import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseFlatTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;

import java.io.IOException;

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

   /**
    * Doesn't work with Valkyrie yet. Need to get it working some day.
    */
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 40.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   @Test(timeout = 300000)
   public void testWalkingUpToRampWithLongStepsAndOccasionallyStraightKnees() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingUpToRampWithLongStepsAndOccasionallyStraightKnees();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 64.8)
   @Test(timeout = 320000)
   public void testRotatedStepInTheAir() throws SimulationExceededMaximumTimeException
   {
      super.testRotatedStepInTheAir();
   }

   @ContinuousIntegrationTest(estimatedDuration = 85.5, categoriesOverride = IntegrationCategory.SLOW)
   @Test(timeout = 430000)
   public void testSimpleScripts() throws SimulationExceededMaximumTimeException, IOException
   {
      super.testSimpleScripts();
   }

   @ContinuousIntegrationTest(estimatedDuration = 59.7, categoriesOverride = IntegrationCategory.SLOW)
   @Test(timeout = 300000)
   public void testACoupleStepsUsingQueuedControllerCommands() throws SimulationExceededMaximumTimeException
   {
      super.testACoupleStepsUsingQueuedControllerCommands();
   }

   @ContinuousIntegrationTest(estimatedDuration = 78.7, categoriesOverride = IntegrationCategory.SLOW)
   @Test(timeout = 390000)
   public void testACoupleQueuedControllerCommands() throws SimulationExceededMaximumTimeException
   {
      super.testACoupleQueuedControllerCommands();
   }

   @ContinuousIntegrationTest(estimatedDuration = 76.7, categoriesOverride = IntegrationCategory.SLOW)
   @Test(timeout = 380000)
   public void testACoupleMoreQueuedControllerCommands() throws SimulationExceededMaximumTimeException
   {
      super.testACoupleMoreQueuedControllerCommands();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 91.2)
   @Test(timeout = 460000)
   public void testWalkingUpToRampWithShortSteps() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingUpToRampWithShortSteps();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 30.1)
   @Test(timeout = 150000)
   public void testStandingTooHighToCheckIfSingularityStuffIsWorkingProperly() throws SimulationExceededMaximumTimeException
   {
      super.testStandingTooHighToCheckIfSingularityStuffIsWorkingProperly();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 25.4, categoriesOverride = IntegrationCategory.SLOW)
   @Test(timeout = 130000)
   public void testStandingForACoupleSeconds() throws SimulationExceededMaximumTimeException
   {
      super.testStandingForACoupleSeconds();
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
   @ContinuousIntegrationTest(estimatedDuration = 272.9, categoriesOverride = IntegrationCategory.MANUAL)
   @Test(timeout = 1400000)
   public void testForMemoryLeaks() throws Exception
   {
      super.testForMemoryLeaks();
   }
}
