package us.ihmc.zulu.obstacleCourseTests;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.zulu.ZuluVersion;
import us.ihmc.zulu.ZuluRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseFlatTest;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

import java.io.IOException;

public class ZuluObstacleCourseFlatTest extends DRCObstacleCourseFlatTest
{
   private final DRCRobotModel robotModel = new ZuluRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(CITools.SimpleRobotNameKeys.ALEXANDER);
   }

   @Tag("humanoid-flat-ground-slow-4")
   @Override
   @Test
   public void testACoupleMoreQueuedControllerCommands()
   {
      super.testACoupleMoreQueuedControllerCommands();
   }

   @Tag("humanoid-flat-ground-slow-4")
   @Override
   @Test
   public void testACoupleQueuedControllerCommands()
   {
      super.testACoupleQueuedControllerCommands();
   }

   @Tag("humanoid-flat-ground-slow-4")
   @Override
   @Test
   public void testACoupleStepsUsingQueuedControllerCommands()
   {
      super.testACoupleStepsUsingQueuedControllerCommands();
   }

   @Tag("humanoid-flat-ground")
   @Override
   // Invoked manually to test memory & thread leaks
   @Disabled
   @Test
   public void testForMemoryLeaks() throws Exception
   {
      super.testForMemoryLeaks();
   }

   @Tag("humanoid-flat-ground-slow-4")
   @Override
   @Test
   public void testLongStepsMaxHeightPauseAndResume()
   {
      super.testLongStepsMaxHeightPauseAndResume();
   }

   @Tag("humanoid-flat-ground-slow-4")
   @Override
   @Test
   public void testRotatedStepInTheAir()
   {
      super.testRotatedStepInTheAir();
   }

   @Tag("humanoid-flat-ground")
   @Override
   @Test
   public void testSimpleScripts() throws IOException
   {
      super.testSimpleScripts();
   }

   @Tag("humanoid-flat-ground-slow-4")
   @Override
   @Test
   public void testStandingForACoupleSeconds()
   {
      super.testStandingForACoupleSeconds();
   }

   @Tag("humanoid-flat-ground")
   @Override
   // TODO re-enable that test when we have polygon to polygon contact model for SCS
   @Disabled
   @Test
   public void testStandingOnUnevenTerrainForACoupleSeconds()
   {
      super.testStandingOnUnevenTerrainForACoupleSeconds();
   }

   @Tag("humanoid-flat-ground-slow-4")
   @Override
   @Test
   public void testStandingTooHighToCheckIfSingularityStuffIsWorkingProperly()
   {
      super.testStandingTooHighToCheckIfSingularityStuffIsWorkingProperly();
   }

   @Tag("humanoid-flat-ground-slow-4")
   @Override
   @Test
   public void testTurningInPlaceAndPassingPI()
   {
      super.testTurningInPlaceAndPassingPI();
   }

   @Tag("humanoid-flat-ground-slow-4")
   @Override
   @Test
   public void testWalkingUpToRampWithLongStepsAndOccasionallyStraightKnees()
   {
      super.testWalkingUpToRampWithLongStepsAndOccasionallyStraightKnees();
   }

   @Tag("humanoid-flat-ground-slow-4")
   @Override
   @Test
   public void testWalkingUpToRampWithShortSteps()
   {
      super.testWalkingUpToRampWithShortSteps();
   }

   @Tag("humanoid-flat-ground-slow-4")
   @Override
   @Test
   public void testRepeatedWalking()
   {
      super.testRepeatedWalking();
   }

   @Tag("humanoid-flat-ground-slow-4")
   @Override
   @Test
   public void testPrepareForLocomotion()
   {
      super.testPrepareForLocomotion();
   }

   @Tag("fast")
   @Override
   @Test
   public void testSimpleFlatGroundScript()
   {
      super.testSimpleFlatGroundScript();
   }
}
