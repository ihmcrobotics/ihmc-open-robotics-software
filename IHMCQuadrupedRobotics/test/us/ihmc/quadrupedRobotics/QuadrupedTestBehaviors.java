package us.ihmc.quadrupedRobotics;

import junit.framework.AssertionFailedError;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerRequestedEvent;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerState;
import us.ihmc.robotics.testing.YoVariableTestGoal;

public class QuadrupedTestBehaviors
{
   public static void standUp(QuadrupedTestConductor conductor, QuadrupedTestYoVariables variables) throws AssertionFailedError
   {
      conductor.addTerminalGoal(YoVariableTestGoal.enumEquals(variables.getForceControllerState(), QuadrupedForceControllerState.STAND_READY));
      conductor.simulate();
      
      variables.getUserTrigger().set(QuadrupedForceControllerRequestedEvent.REQUEST_STAND);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyZ(), 0.58));
      conductor.addTerminalGoal(YoVariableTestGoal.enumEquals(variables.getForceControllerState(), QuadrupedForceControllerState.STAND));
      conductor.simulate();
   }
}
