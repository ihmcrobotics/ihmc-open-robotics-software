package us.ihmc.quadrupedRobotics;

import junit.framework.AssertionFailedError;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerRequestedEvent;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerState;
import us.ihmc.quadrupedRobotics.controller.position.QuadrupedPositionControllerRequestedEvent;
import us.ihmc.quadrupedRobotics.controller.position.QuadrupedPositionControllerState;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;

public class QuadrupedTestBehaviors
{
   public static void readyXGait(GoalOrientedTestConductor conductor, QuadrupedForceTestYoVariables variables) throws AssertionFailedError
   {
      standUp(conductor, variables);
      squareUp(conductor, variables);
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
      conductor.addTimeLimit(variables.getYoTime(), 2.0);
      conductor.addTerminalGoal(YoVariableTestGoal.enumEquals(variables.getForceControllerState(), QuadrupedForceControllerState.XGAIT));
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 1.0));
      conductor.simulate();
   }

   private static void standUp(GoalOrientedTestConductor conductor, QuadrupedForceTestYoVariables variables) throws AssertionFailedError
   {
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 0.2));
      conductor.simulate();

      variables.getUserTrigger().set(QuadrupedForceControllerRequestedEvent.REQUEST_STAND);
      conductor.addTerminalGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), 2.0);
      //      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getComPositionEstimateZ(), variables.getYoComPositionInputZ().getDoubleValue(), 0.05)); // doesn't work well when not flat ground - BS
      conductor.addTerminalGoal(YoVariableTestGoal.enumEquals(variables.getForceControllerState(), QuadrupedForceControllerState.STAND));
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 1.0));
      conductor.simulate();
   }

   private static void squareUp(GoalOrientedTestConductor conductor, QuadrupedForceTestYoVariables variables)
   {
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 0.2));
      conductor.simulate();
      
      double initialDoubleSupportDuration = variables.getXGaitEndDoubleSupportDurationInput().getDoubleValue();
      double initialEndPhaseShift = variables.getXGaitEndPhaseShiftInput().getDoubleValue();
      
      variables.getXGaitEndDoubleSupportDurationInput().set(0.1);
      variables.getXGaitEndPhaseShiftInput().set(180.0);
      variables.getUserTrigger().set(QuadrupedForceControllerRequestedEvent.REQUEST_XGAIT);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 1.5));
      conductor.simulate();

      variables.getUserTrigger().set(QuadrupedForceControllerRequestedEvent.REQUEST_STAND);
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 1.0));
      conductor.simulate();
      
      variables.getXGaitEndDoubleSupportDurationInput().set(initialDoubleSupportDuration);
      variables.getXGaitEndPhaseShiftInput().set(initialEndPhaseShift);
   }
}
