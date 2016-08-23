package us.ihmc.quadrupedRobotics;

import junit.framework.AssertionFailedError;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerRequestedEvent;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerState;
import us.ihmc.quadrupedRobotics.controller.position.QuadrupedPositionControllerRequestedEvent;
import us.ihmc.quadrupedRobotics.controller.position.QuadrupedPositionControllerState;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationconstructionset.util.simulationRunner.GoalOrientedTestConductor;

public class QuadrupedTestBehaviors
{
   public static void standUp(GoalOrientedTestConductor conductor, QuadrupedForceTestYoVariables variables) throws AssertionFailedError
   {
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.enumEquals(variables.getForceControllerState(), QuadrupedForceControllerState.STAND_READY));
      conductor.simulate();

      variables.getUserTrigger().set(QuadrupedForceControllerRequestedEvent.REQUEST_STAND);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), 3.0);
//      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getComPositionEstimateZ(), variables.getYoComPositionInputZ().getDoubleValue(), 0.05)); // doesn't work well when not flat ground - BS
      conductor.addTerminalGoal(YoVariableTestGoal.enumEquals(variables.getForceControllerState(), QuadrupedForceControllerState.STAND));
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 1.0));
      conductor.simulate();
   }

   public static void standUp(GoalOrientedTestConductor conductor, QuadrupedPositionTestYoVariables variables) throws AssertionFailedError
   {
      conductor.addTerminalGoal(YoVariableTestGoal.enumEquals(variables.getPositionControllerState(), QuadrupedPositionControllerState.DO_NOTHING));
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 1.0));
      conductor.simulate();

      variables.getUserTrigger().set(QuadrupedPositionControllerRequestedEvent.REQUEST_STAND_PREP);
      conductor.addTerminalGoal(YoVariableTestGoal.enumEquals(variables.getPositionControllerState(), QuadrupedPositionControllerState.STAND_READY));
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 2.0));
      conductor.simulate();

      variables.getUserTrigger().set(QuadrupedPositionControllerRequestedEvent.REQUEST_CRAWL);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyZ(), variables.getYoComPositionInputZ().getDoubleValue(), 0.1));
      conductor.addTerminalGoal(YoVariableTestGoal.enumEquals(variables.getPositionControllerState(), QuadrupedPositionControllerState.CRAWL));
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 2.0));
      conductor.simulate();
   }

   public static void enterXGait(GoalOrientedTestConductor conductor, QuadrupedForceTestYoVariables variables) throws AssertionFailedError
   {
      variables.getUserTrigger().set(QuadrupedForceControllerRequestedEvent.REQUEST_XGAIT);
      conductor.addTerminalGoal(YoVariableTestGoal.enumEquals(variables.getForceControllerState(), QuadrupedForceControllerState.XGAIT));
      conductor.simulate();
   }
}
