package us.ihmc.valkyrie.obstacleCourse;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseRampsTest;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieObstacleCourseRampsTest extends DRCObstacleCourseRampsTest
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

   @Tag("humanoid-rough-terrain")
   @Override
   @Test
   public void testWalkingDownRampWithMediumSteps() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingDownRampWithMediumSteps();
   }

   @Tag("humanoid-rough-terrain")
   @Override
   @Test
   public void testWalkingUpRampWithMediumSteps() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingUpRampWithMediumSteps();
   }

   @Tag("humanoid-obstacle-slow")
   @Override
   @Test
   public void testWalkingUpRampWithShortSteps() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingUpRampWithShortSteps();
   }

   @Tag("humanoid-obstacle-slow")
   @Override
   @Test
   public void testWalkingUpRampWithShortStepsALittleTooHigh() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingUpRampWithShortStepsALittleTooHigh();
   }

   @Tag("humanoid-obstacle-slow")
   @Override
   @Test
   public void testWalkingUpRampWithShortStepsALittleTooLow() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingUpRampWithShortStepsALittleTooLow();
   }

   @Disabled // FIXME This test often hangs on Bamboo. 
   @Override
   @Test
   public void testHeightReinitialization() throws SimulationExceededMaximumTimeException
   {
      super.testHeightReinitialization();
   }

   @Override
   protected double getMaxRotationCorruption()
   {
      return 0.0;
   }
}
