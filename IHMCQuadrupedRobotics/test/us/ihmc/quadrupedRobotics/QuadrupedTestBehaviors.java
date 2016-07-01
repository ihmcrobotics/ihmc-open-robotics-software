package us.ihmc.quadrupedRobotics;

import junit.framework.AssertionFailedError;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerRequestedEvent;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerState;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationconstructionset.util.simulationRunner.GoalOrientedTestConductor;

public class QuadrupedTestBehaviors
{
   public static void standUp(GoalOrientedTestConductor conductor, QuadrupedTestYoVariables variables) throws AssertionFailedError
   {
      conductor.addTerminalGoal(YoVariableTestGoal.enumEquals(variables.getForceControllerState(), QuadrupedForceControllerState.STAND_READY));
      conductor.simulate();
      
      variables.getUserTrigger().set(QuadrupedForceControllerRequestedEvent.REQUEST_STAND);
      conductor.addSustainGoal(YoVariableTestGoal.doubleLessThan(variables.getYoTime(), 2.0));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyZ(), variables.getYoComPositionInputZ().getDoubleValue(), 0.1));
      conductor.addTerminalGoal(YoVariableTestGoal.enumEquals(variables.getForceControllerState(), QuadrupedForceControllerState.STAND));
      conductor.simulate();
   }
}
