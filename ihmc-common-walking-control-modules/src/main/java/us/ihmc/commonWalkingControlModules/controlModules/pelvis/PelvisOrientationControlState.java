package us.ihmc.commonWalkingControlModules.controlModules.pelvis;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.robotics.stateMachine.core.State;

public interface PelvisOrientationControlState extends State
{
   @Override
   default boolean isDone(double timeInState)
   {
      return true;
   }

   default InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return null;
   }

   @Override
   default void onEntry()
   {
   }

   @Override
   default void onExit()
   {
   }

   public abstract void goToHomeFromCurrentDesired(double trajectoryTime);

   public abstract FeedbackControlCommand<?> getFeedbackControlCommand();

   public abstract void getCurrentDesiredOrientation(FrameQuaternion orientationToPack);
}
