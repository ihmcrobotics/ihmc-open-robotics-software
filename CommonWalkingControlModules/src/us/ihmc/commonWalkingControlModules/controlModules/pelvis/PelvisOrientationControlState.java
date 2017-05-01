package us.ihmc.commonWalkingControlModules.controlModules.pelvis;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
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

   public abstract OrientationFeedbackControlCommand getFeedbackControlCommand();

   public abstract void getCurrentDesiredOrientation(FrameOrientation orientationToPack);
}
