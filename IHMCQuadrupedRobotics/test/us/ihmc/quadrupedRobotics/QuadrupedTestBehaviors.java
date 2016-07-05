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
   public static void standUp(GoalOrientedTestConductor conductor, QuadrupedTestYoVariables variables) throws AssertionFailedError
   {
      if (variables instanceof QuadrupedForceTestYoVariables)
      {
         QuadrupedForceTestYoVariables forceVariables = (QuadrupedForceTestYoVariables) variables;
         
         conductor.addTerminalGoal(YoVariableTestGoal.enumEquals(forceVariables.getForceControllerState(), QuadrupedForceControllerState.STAND_READY));
         conductor.simulate();
         
         forceVariables.getUserTrigger().set(QuadrupedForceControllerRequestedEvent.REQUEST_STAND);
         conductor.addSustainGoal(YoVariableTestGoal.doubleLessThan(variables.getYoTime(), 2.0));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyZ(), variables.getYoComPositionInputZ().getDoubleValue(), 0.1));
         conductor.addTerminalGoal(YoVariableTestGoal.enumEquals(forceVariables.getForceControllerState(), QuadrupedForceControllerState.STAND));
         conductor.simulate();
      }
      else
      {
         QuadrupedPositionTestYoVariables positionVariables = (QuadrupedPositionTestYoVariables) variables;
         conductor.addTerminalGoal(YoVariableTestGoal.enumEquals(positionVariables.getPositionControllerState(), QuadrupedPositionControllerState.DO_NOTHING));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
         conductor.simulate();
         
         positionVariables.getUserTrigger().set(QuadrupedPositionControllerRequestedEvent.REQUEST_STAND_PREP);
         conductor.addTerminalGoal(YoVariableTestGoal.enumEquals(positionVariables.getPositionControllerState(), QuadrupedPositionControllerState.STAND_READY));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 2.0));
         conductor.simulate();
         
         positionVariables.getUserTrigger().set(QuadrupedPositionControllerRequestedEvent.REQUEST_CRAWL);
         conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyZ(), variables.getYoComPositionInputZ().getDoubleValue(), 0.1));
         conductor.addTerminalGoal(YoVariableTestGoal.enumEquals(positionVariables.getPositionControllerState(), QuadrupedPositionControllerState.CRAWL));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 2.0));
         conductor.simulate();
      }
   }
}
