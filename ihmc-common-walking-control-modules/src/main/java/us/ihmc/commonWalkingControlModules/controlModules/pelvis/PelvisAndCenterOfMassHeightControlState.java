package us.ihmc.commonWalkingControlModules.controlModules.pelvis;

import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.robotics.stateMachine.core.State;

public interface PelvisAndCenterOfMassHeightControlState extends State
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

   public abstract FeedbackControlCommand<?> getFeedbackControlCommand();

   /**
    * This method is intended to reset the internal state of this control state to be identical to
    * when starting up the whole controller.
    * <p>
    * This allows to re-initialize the walking controller.
    * </p>
    */
   public abstract void initialize();

   public abstract void initializeDesiredHeightToCurrent();

   public abstract void goHome(double trajectoryTime);

   public abstract void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command);

   public abstract double computeDesiredCoMHeightAcceleration(FrameVector2D desiredICPVelocity, boolean isInDoubleSupport, double omega0,
                                                              boolean isRecoveringFromPush, FeetManager feetManager);

   default TaskspaceTrajectoryStatusMessage pollStatusToReport()
   {
      return null;
   }
}
