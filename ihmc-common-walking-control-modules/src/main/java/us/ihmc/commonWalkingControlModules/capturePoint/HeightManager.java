package us.ihmc.commonWalkingControlModules.capturePoint;

import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisHeightTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.providers.DoubleProvider;

public interface HeightManager
{

   public abstract void setSupportLeg(RobotSide oppositeSide);

   public abstract void initialize(TransferToAndNextFootstepsData transferToAndNextFootstepsData, double extraToeOffHeight);

   public abstract void initializeToNominalDesiredHeight();

   public abstract void prepareForLocomotion();

   public abstract void initializeTransitionToFall(double d);

   public abstract void handlePelvisHeightTrajectoryCommand(PelvisHeightTrajectoryCommand command);

   public abstract void goHome(double trajectoryTime);

   public abstract void handlePelvisTrajectoryCommand(PelvisTrajectoryCommand command);

   public abstract void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command);

   public abstract void setPrepareForLocomotion(boolean preparePelvis);

   public abstract void initialize();

   public abstract void initializeDesiredHeightToCurrent();

   public abstract void compute(FrameVector2DReadOnly desiredICPVelocity,
                       FrameVector2DReadOnly desiredCoMVelocity,
                       boolean isInDoubleSupport,
                       double omega0,
                       FeetManager feetManager);

   public abstract FeedbackControlCommand<?> getHeightControlCommand();

   public abstract boolean getControlHeightWithMomentum();

   public abstract TaskspaceTrajectoryStatusMessage pollStatusToReport();

   public abstract FeedbackControlCommand<?> getFeedbackControlCommand();

   public abstract void setPelvisTaskspaceWeights(Vector3DReadOnly pelvisLinearWeight);

   public abstract void setComHeightGains(PIDGainsReadOnly walkingControllerComHeightGains,
                                          DoubleProvider walkingControllerMaxComHeightVelocity,
                                          PIDGainsReadOnly userModeComHeightGains);

   public abstract FeedbackControlCommand<?> createFeedbackControlTemplate();

}
