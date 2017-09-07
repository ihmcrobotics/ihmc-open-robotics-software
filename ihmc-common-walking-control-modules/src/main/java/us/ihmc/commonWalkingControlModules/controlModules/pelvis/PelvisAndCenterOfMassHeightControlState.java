package us.ihmc.commonWalkingControlModules.controlModules.pelvis;

import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.FinishableState;

public abstract class PelvisAndCenterOfMassHeightControlState extends FinishableState<PelvisHeightControlMode>
{

   public PelvisAndCenterOfMassHeightControlState(PelvisHeightControlMode stateEnum)
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

   public abstract FeedbackControlCommand<?> getFeedbackControlCommand();

   public abstract void getCurrentDesiredHeightOfDefaultControlFrame(FramePoint3D positionToPack);

   public abstract void initializeDesiredHeightToCurrent();

   public abstract void goHome(double trajectoryTime);

   public abstract void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command);

   public abstract double computeDesiredCoMHeightAcceleration(FrameVector2D desiredICPVelocity, boolean isInDoubleSupport, double omega0, boolean isRecoveringFromPush,
         FeetManager feetManager);
}
