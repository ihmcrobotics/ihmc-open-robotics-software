package us.ihmc.valkyrie.obstacleCourse;

import java.io.IOException;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseFlatTest;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieObstacleCourseFlatTest extends DRCObstacleCourseFlatTest
{
   private final DRCRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS);

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
   @Tag("humanoid-flat-ground")
   @Override
   @Disabled
   @Test
   public void testWalkingUpToRampWithLongStepsAndOccasionallyStraightKnees()
   {
      super.testWalkingUpToRampWithLongStepsAndOccasionallyStraightKnees();
   }

   @Tag("humanoid-flat-ground-slow")
   @Override
   @Test
   public void testRotatedStepInTheAir()
   {
      super.testRotatedStepInTheAir();
   }

   @Tag("humanoid-flat-ground")
   @Test
   public void testSimpleScripts() throws IOException
   {
      super.testSimpleScripts();
   }

   @Tag("humanoid-flat-ground-slow")
   @Test
   public void testACoupleStepsUsingQueuedControllerCommands()
   {
      super.testACoupleStepsUsingQueuedControllerCommands();
   }

   @Tag("humanoid-flat-ground-slow")
   @Test
   public void testACoupleQueuedControllerCommands()
   {
      super.testACoupleQueuedControllerCommands();
   }

   @Tag("humanoid-flat-ground-slow")
   @Test
   public void testACoupleMoreQueuedControllerCommands()
   {
      super.testACoupleMoreQueuedControllerCommands();
   }

   @Tag("humanoid-flat-ground-slow")
   @Override
   @Test
   public void testWalkingUpToRampWithShortSteps()
   {
      super.testWalkingUpToRampWithShortSteps();
   }

   @Tag("humanoid-flat-ground-slow")
   @Override
   @Test
   public void testStandingTooHighToCheckIfSingularityStuffIsWorkingProperly()
   {
      super.testStandingTooHighToCheckIfSingularityStuffIsWorkingProperly();
   }

   @Tag("humanoid-flat-ground-slow")
   @Override
   @Test
   public void testStandingForACoupleSeconds()
   {
      super.testStandingForACoupleSeconds();
   }

   @Tag("humanoid-flat-ground-slow")
   @Override
   @Test
   public void testLongStepsMaxHeightPauseAndResume()
   {
      super.testLongStepsMaxHeightPauseAndResume();
   }

   @Tag("humanoid-flat-ground-slow")
   @Override
   @Test
   public void testTurningInPlaceAndPassingPI()
   {
      super.testTurningInPlaceAndPassingPI();
   }

   @Tag("fast")
   @Disabled // FIXME That test is quite pointless
   @Override
   @Test
   public void testStandingOnUnevenTerrainForACoupleSeconds()
   {
      super.testStandingOnUnevenTerrainForACoupleSeconds();
   }

   @Tag("humanoid-flat-ground")
   @Override
   @Disabled
   @Test
   public void testForMemoryLeaks() throws Exception
   {
      super.testForMemoryLeaks();
   }

   @Tag("humanoid-flat-ground-slow")
   @Override
   @Test
   public void testPrepareForLocomotion()
   {
      super.testPrepareForLocomotion();
   }

   @Tag("humanoid-flat-ground")
   @Override
   @Test
   public void testRepeatedWalking()
   {
      super.testRepeatedWalking();
   }
}

