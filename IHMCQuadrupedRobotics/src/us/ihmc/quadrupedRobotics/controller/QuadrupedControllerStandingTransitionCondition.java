package us.ihmc.quadrupedRobotics.controller;

import us.ihmc.robotics.stateMachines.StateTransitionCondition;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;

/**
 * A transition condition that transitions when the target controller's motion status becomes #{@link
 * RobotMotionStatus#STANDING}.
 */
public class QuadrupedControllerStandingTransitionCondition implements StateTransitionCondition
{
   private final QuadrupedController controller;

   public QuadrupedControllerStandingTransitionCondition(QuadrupedController controller)
   {
      this.controller = controller;
   }

   @Override
   public boolean checkCondition()
   {
      return controller.getMotionStatus() == RobotMotionStatus.STANDING;
   }
}
