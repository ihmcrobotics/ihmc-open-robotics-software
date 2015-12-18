package us.ihmc.quadrupedRobotics.controller;

import us.ihmc.sensorProcessing.model.RobotMotionStatus;

/**
 * A dummy controller that merely signifies that the robot is prepared to transition
 * to another controller.
 */
public class QuadrupedStandReadyController extends QuadrupedController
{
   public QuadrupedStandReadyController()
   {
      super(QuadrupedControllerState.STAND_READY);
   }

   @Override
   public void doAction()
   {
      // Do nothing.
   }

   @Override
   public void doTransitionIntoAction()
   {
   }

   @Override
   public void doTransitionOutOfAction()
   {
   }

   @Override
   public RobotMotionStatus getMotionStatus()
   {
      return RobotMotionStatus.STANDING;
   }
}
