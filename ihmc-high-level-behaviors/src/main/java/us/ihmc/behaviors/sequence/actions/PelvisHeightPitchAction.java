package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ActionExecutionStatusMessage;
import behavior_msgs.msg.dds.BodyPartPoseStatusMessage;
import controller_msgs.msg.dds.PelvisTrajectoryMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.BehaviorAction;
import us.ihmc.behaviors.sequence.BehaviorActionCompletionCalculator;
import us.ihmc.behaviors.sequence.BehaviorActionCompletionComponent;
import us.ihmc.behaviors.sequence.BehaviorActionSequence;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.Timer;

public class PelvisHeightPitchAction extends PelvisHeightPitchActionData implements BehaviorAction
{
   public static final double POSITION_TOLERANCE = 0.15;
   public static final double ORIENTATION_TOLERANCE = Math.toRadians(10.0);

   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private int actionIndex;
   private final BodyPartPoseStatusMessage pelvisPoseStatus = new BodyPartPoseStatusMessage();
   private final Timer executionTimer = new Timer();
   private boolean isExecuting;
   private final FramePose3D desiredPelvisPose = new FramePose3D();
   private final FramePose3D syncedPelvisPose = new FramePose3D();
   private double startPositionDistanceToGoal;
   private double startOrientationDistanceToGoal;
   private final ActionExecutionStatusMessage executionStatusMessage = new ActionExecutionStatusMessage();
   private final BehaviorActionCompletionCalculator completionCalculator = new BehaviorActionCompletionCalculator();

   public PelvisHeightPitchAction(ROS2ControllerHelper ros2ControllerHelper,
                                  ReferenceFrameLibrary referenceFrameLibrary,
                                  ROS2SyncedRobotModel syncedRobot)
   {
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;
      setReferenceFrameLibrary(referenceFrameLibrary);
   }

   @Override
   public void update(int actionIndex, int nextExecutionIndex, boolean concurrencyWithPreviousIndex, int indexShiftConcurrentAction)
   {
      update();

      this.actionIndex = actionIndex;

      // if the action is part of a group of concurrent actions that is currently executing or about to be executed
      if ((concurrencyWithPreviousIndex && actionIndex == (nextExecutionIndex + indexShiftConcurrentAction)) ||
          (getExecuteWithNextAction() && actionIndex == nextExecutionIndex))
      {
         pelvisPoseStatus.getParentFrame().resetQuick();
         pelvisPoseStatus.getParentFrame().add(getParentFrame().getName());
         MessageTools.toMessage(getTransformToParent(), pelvisPoseStatus.getTransformToParent());
         ros2ControllerHelper.publish(BehaviorActionSequence.PELVIS_POSITION_STATUS, pelvisPoseStatus);
      }
   }

   @Override
   public void triggerActionExecution()
   {
      FramePose3D framePose = new FramePose3D(getReferenceFrame());
      FramePose3D syncedPose = new FramePose3D(syncedRobot.getFullRobotModel().getPelvis().getBodyFixedFrame());
      framePose.getRotation().setYawPitchRoll(syncedPose.getYaw(), framePose.getPitch(), syncedPose.getRoll());
      framePose.changeFrame(ReferenceFrame.getWorldFrame());
      syncedPose.changeFrame(ReferenceFrame.getWorldFrame());

      PelvisTrajectoryMessage message = new PelvisTrajectoryMessage();
      message.getSe3Trajectory()
             .set(HumanoidMessageTools.createSE3TrajectoryMessage(getTrajectoryDuration(),
                                                                        framePose.getPosition(),
                                                                        framePose.getOrientation(),
                                                                        ReferenceFrame.getWorldFrame()));
      long frameId = MessageTools.toFrameId(ReferenceFrame.getWorldFrame());
      message.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(frameId);
      message.getSe3Trajectory().getLinearSelectionMatrix().setXSelected(false);
      message.getSe3Trajectory().getLinearSelectionMatrix().setYSelected(false);
      message.getSe3Trajectory().getLinearSelectionMatrix().setZSelected(true);
      ros2ControllerHelper.publishToController(message);
      executionTimer.reset();

      desiredPelvisPose.setFromReferenceFrame(getReferenceFrame());
      syncedPelvisPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getPelvis().getBodyFixedFrame());
      desiredPelvisPose.getTranslation().set(syncedPelvisPose.getTranslationX(), syncedPelvisPose.getTranslationY(), desiredPelvisPose.getTranslationZ());
      desiredPelvisPose.getRotation().setYawPitchRoll(syncedPelvisPose.getYaw(), desiredPelvisPose.getPitch(), syncedPelvisPose.getRoll());
      startPositionDistanceToGoal = syncedPelvisPose.getTranslation().differenceNorm(desiredPelvisPose.getTranslation());
      startOrientationDistanceToGoal = syncedPelvisPose.getRotation().distance(desiredPelvisPose.getRotation(), true);
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      desiredPelvisPose.setFromReferenceFrame(getReferenceFrame());
      syncedPelvisPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getPelvis().getBodyFixedFrame());
      desiredPelvisPose.getTranslation().set(syncedPelvisPose.getTranslationX(), syncedPelvisPose.getTranslationY(), desiredPelvisPose.getTranslationZ());
      desiredPelvisPose.getRotation().setYawPitchRoll(syncedPelvisPose.getYaw(), desiredPelvisPose.getPitch(), syncedPelvisPose.getRoll());

      isExecuting = !completionCalculator.isComplete(desiredPelvisPose,
                                                     syncedPelvisPose,
                                                     POSITION_TOLERANCE, Double.NaN,
                                                     getTrajectoryDuration(),
                                                     executionTimer,
                                                     BehaviorActionCompletionComponent.TRANSLATION);

      executionStatusMessage.setActionIndex(actionIndex);
      executionStatusMessage.setNominalExecutionDuration(getTrajectoryDuration());
      executionStatusMessage.setElapsedExecutionTime(executionTimer.getElapsedTime());
      executionStatusMessage.setStartOrientationDistanceToGoal(startOrientationDistanceToGoal);
      executionStatusMessage.setStartPositionDistanceToGoal(startPositionDistanceToGoal);
      executionStatusMessage.setCurrentOrientationDistanceToGoal(completionCalculator.getRotationError());
      executionStatusMessage.setCurrentPositionDistanceToGoal(completionCalculator.getTranslationError());
      executionStatusMessage.setPositionDistanceToGoalTolerance(POSITION_TOLERANCE);
      executionStatusMessage.setOrientationDistanceToGoalTolerance(ORIENTATION_TOLERANCE);
   }

   @Override
   public ActionExecutionStatusMessage getExecutionStatusMessage()
   {
      return executionStatusMessage;
   }

   @Override
   public boolean isExecuting()
   {
      return isExecuting;
   }
}
