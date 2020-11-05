package us.ihmc.avatar.jumping;

import org.junit.jupiter.api.Test;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.JumpingGoal;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertFalse;

public abstract class AvatarStandingLongJumpTests
{
   protected abstract DRCRobotModel getRobotModel();

   protected abstract DRCRobotInitialSetup getInitialSetup();

   protected abstract String getParameterResourceName();

   @Test
   public void testStandingShortJump() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      JumpingTestHelper testHelper = new JumpingTestHelper(getRobotModel(), getInitialSetup(), getParameterResourceName());
      CommandInputManager commandInputManager = testHelper.getCommandInputManager();

      YoRegistry registry = testHelper.getSCS().getRootRegistry();
      YoBoolean shouldBeSquatting = ((YoBoolean) registry.findVariable("ShouldBeSquatting"));
      YoDouble comX = ((YoDouble) registry.findVariable("centerOfMassX"));
      YoDouble comY = ((YoDouble) registry.findVariable("centerOfMassY"));
      YoDouble comZ = ((YoDouble) registry.findVariable("centerOfMassZ"));

      BlockingSimulationRunner blockingSimulationRunner = testHelper.getBlockingSimulationRunner();

      shouldBeSquatting.set(true);
      boolean success = blockingSimulationRunner.simulateAndBlockAndCatchExceptions(2.0);

      double length = 0.25;
      JumpingGoal jumpingGoal = new JumpingGoal();
      jumpingGoal.setGoalLength(length);
      jumpingGoal.setSupportDuration(0.25);
      jumpingGoal.setFlightDuration(0.4);
      commandInputManager.submitCommand(jumpingGoal);

      success &= blockingSimulationRunner.simulateAndBlockAndCatchExceptions(2.0);
      assertFalse(success);

      assertEquals(length, comX.getDoubleValue(), 1e-2);
      assertEquals(0.0, comY.getDoubleValue(), 1e-2);
   }
}
