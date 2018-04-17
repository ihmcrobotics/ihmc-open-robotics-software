package us.ihmc.quadrupedRobotics;

import junit.framework.AssertionFailedError;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerRequestedEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerEnum;
import us.ihmc.quadrupedRobotics.controller.QuadrupedSteppingStateEnum;
import us.ihmc.quadrupedRobotics.controller.QuadrupedPositionControllerRequestedEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedPositionControllerState;
import us.ihmc.quadrupedRobotics.input.managers.QuadrupedStepTeleopManager;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;

public class QuadrupedTestBehaviors
{
   public static void readyXGait(GoalOrientedTestConductor conductor, QuadrupedForceTestYoVariables variables, QuadrupedStepTeleopManager stepTeleopManager) throws AssertionFailedError
   {
      standUp(conductor, variables);
      startBalancing(conductor, variables, stepTeleopManager);
      squareUp(conductor, variables, stepTeleopManager);
   }

   public static void standUp(GoalOrientedTestConductor conductor, QuadrupedForceTestYoVariables variables) throws AssertionFailedError
   {
      variables.getUserTrigger().set(QuadrupedControllerRequestedEvent.REQUEST_FREEZE);
      conductor.addTerminalGoal(YoVariableTestGoal.enumEquals(variables.getControllerState(), QuadrupedControllerEnum.FREEZE));
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 0.5));
      conductor.simulate();

      variables.getUserTrigger().set(QuadrupedControllerRequestedEvent.REQUEST_STAND_PREP);
      conductor.addTerminalGoal(YoVariableTestGoal.enumEquals(variables.getControllerState(), QuadrupedControllerEnum.STAND_READY));
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

   public static void enterXGait(GoalOrientedTestConductor conductor, QuadrupedForceTestYoVariables variables, QuadrupedStepTeleopManager stepTeleopManager) throws AssertionFailedError
   {
      stepTeleopManager.setDesiredVelocity(0.0, 0.0, 0.0);
      stepTeleopManager.requestXGait();
      conductor.addTimeLimit(variables.getYoTime(), 2.0);
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 1.0));
      conductor.simulate();
   }

   public static void startBalancing(GoalOrientedTestConductor conductor, QuadrupedForceTestYoVariables variables, QuadrupedStepTeleopManager stepTeleopManager) throws AssertionFailedError
   {
      stepTeleopManager.requestSteppingState();
      conductor.addTerminalGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(QuadrupedTestGoals.bodyHeight(variables, 0.1));
      conductor.addTimeLimit(variables.getYoTime(), 2.0);
      conductor.addTerminalGoal(YoVariableTestGoal.enumEquals(variables.getControllerState(), QuadrupedControllerEnum.STEPPING));
      conductor.addTerminalGoal(YoVariableTestGoal.enumEquals(variables.getSteppingState(), QuadrupedSteppingStateEnum.STAND));
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 1.0));
      conductor.simulate();
   }

   public static void squareUp(GoalOrientedTestConductor conductor, QuadrupedForceTestYoVariables variables, QuadrupedStepTeleopManager stepTeleopManager)
   {
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 0.2));
      conductor.addSustainGoal(QuadrupedTestGoals.bodyHeight(variables, 0.25));
      conductor.simulate();

      double initialDoubleSupportDuration = stepTeleopManager.getXGaitSettings().getEndDoubleSupportDuration();
      double initialEndPhaseShift = stepTeleopManager.getXGaitSettings().getEndPhaseShift();

      stepTeleopManager.getXGaitSettings().setEndDoubleSupportDuration(0.1);
      stepTeleopManager.getXGaitSettings().setEndPhaseShift(180.0);
      stepTeleopManager.requestXGait();
      stepTeleopManager.setDesiredVelocity(0.0, 0.0, 0.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addSustainGoal(QuadrupedTestGoals.bodyHeight(variables, 0.25));
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 1.5));
      conductor.simulate();

      stepTeleopManager.requestStanding();
      conductor.addSustainGoal(QuadrupedTestGoals.bodyHeight(variables, 0.25));
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 1.0));
      conductor.simulate();

      stepTeleopManager.getXGaitSettings().setEndDoubleSupportDuration(initialDoubleSupportDuration);
      stepTeleopManager.getXGaitSettings().setEndPhaseShift(initialEndPhaseShift);
   }




}
