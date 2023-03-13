package us.ihmc.commonWalkingControlModules.controlModules.pelvis;

import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.stateMachine.core.State;

public interface PelvisAndCenterOfMassHeightControlState extends State, SCS2YoGraphicHolder
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
   default void onExit(double timeInState)
   {
   }

   FeedbackControlCommand<?> getFeedbackControlCommand();

   FeedbackControlCommand<?> getHeightControlCommand();

   default FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      return getFeedbackControlCommand();
   }

   boolean getControlHeightWithMomentum();

   /**
    * This method is intended to reset the internal state of this control state to be identical to
    * when starting up the whole controller.
    * <p>
    * This allows to re-initialize the walking controller.
    * </p>
    */
   void initialize();

   void initializeDesiredHeightToCurrent();

   void goHome(double trajectoryTime);

   void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command);

   void computeCoMHeightCommand(FrameVector2DReadOnly desiredICPVelocity,
                                FrameVector2DReadOnly desiredCoMVelocity,
                                boolean isInDoubleSupport,
                                double omega0,
                                FeetManager feetManager);

   default TaskspaceTrajectoryStatusMessage pollStatusToReport()
   {
      return null;
   }
}
