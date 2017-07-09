package us.ihmc.commonWalkingControlModules.controlModules.pelvis;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.FinishableState;

public abstract class PelvisOrientationControlState extends FinishableState<PelvisOrientationControlMode>
{

   public PelvisOrientationControlState(PelvisOrientationControlMode stateEnum)
   {
      super(stateEnum);
   }

   @Override
   public boolean isDone()
   {
      return true;
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return null;
   }

   @Override
   public void doTransitionIntoAction()
   {
   }

   @Override
   public void doTransitionOutOfAction()
   {
   }

   public abstract void goToHomeFromCurrentDesired(double trajectoryTime);

   public abstract FeedbackControlCommand<?> getFeedbackControlCommand();

   public abstract void getCurrentDesiredOrientation(FrameOrientation orientationToPack);
}
