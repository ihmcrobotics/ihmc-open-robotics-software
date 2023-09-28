package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ActionExecutionStatusMessage;
import controller_msgs.msg.dds.ChestTrajectoryMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.BehaviorAction;
import us.ihmc.behaviors.sequence.BehaviorActionCompletionCalculator;
import us.ihmc.behaviors.sequence.BehaviorActionCompletionComponent;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.Timer;

public class ChestOrientationAction extends ChestOrientationActionDescription implements BehaviorAction
{
   public static final double ORIENTATION_TOLERANCE = Math.toRadians(10.0);

   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private int actionIndex;
   private final Timer executionTimer = new Timer();
   private boolean isExecuting;
   private final FramePose3D desiredChestPose = new FramePose3D();
   private final FramePose3D syncedChestPose = new FramePose3D();
   private double startOrientationDistanceToGoal;
   private final ActionExecutionStatusMessage executionStatusMessage = new ActionExecutionStatusMessage();
   private final BehaviorActionCompletionCalculator completionCalculator = new BehaviorActionCompletionCalculator();
   private double heightVariationInWorld = 0.0;
   private double previousPelvisHeightInWorld = -1.0;

   public ChestOrientationAction(ROS2ControllerHelper ros2ControllerHelper, ROS2SyncedRobotModel syncedRobot, ReferenceFrameLibrary referenceFrameLibrary)
   {
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;
      this.referenceFrameLibrary = referenceFrameLibrary;
   }

   @Override
   public void update(int actionIndex, int nextExecutionIndex, boolean concurrencyWithPreviousIndex, int indexShiftConcurrentAction)
   {
      update(referenceFrameLibrary);

      this.actionIndex = actionIndex;
   }

   @Override
   public void triggerActionExecution()
   {
      FrameQuaternion frameChestQuaternion = new FrameQuaternion(getConditionalReferenceFrame().get());
      frameChestQuaternion.changeFrame(syncedRobot.getReferenceFrames().getPelvisZUpFrame());

      ChestTrajectoryMessage message = new ChestTrajectoryMessage();
      message.getSo3Trajectory()
             .set(HumanoidMessageTools.createSO3TrajectoryMessage(getTrajectoryDuration(),
                                                                  frameChestQuaternion,
                                                                  EuclidCoreTools.zeroVector3D,
                                                                  ReferenceFrame.getWorldFrame()));
      long frameId = MessageTools.toFrameId(ReferenceFrame.getWorldFrame());
      message.getSo3Trajectory().getFrameInformation().setDataReferenceFrameId(frameId);

      ros2ControllerHelper.publishToController(message);
      executionTimer.reset();

      desiredChestPose.setFromReferenceFrame(getConditionalReferenceFrame().get());
      syncedChestPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getChest().getBodyFixedFrame());
      startOrientationDistanceToGoal = syncedChestPose.getRotation().distance(desiredChestPose.getRotation(), true);
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      desiredChestPose.setFromReferenceFrame(getConditionalReferenceFrame().get());
      syncedChestPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getChest().getBodyFixedFrame());

      boolean wasExecuting = isExecuting;
      isExecuting = !completionCalculator.isComplete(desiredChestPose,
                                                     syncedChestPose,
                                                     Double.NaN, ORIENTATION_TOLERANCE,
                                                     getTrajectoryDuration(),
                                                     executionTimer,
                                                     BehaviorActionCompletionComponent.ORIENTATION);

      executionStatusMessage.setActionIndex(actionIndex);
      executionStatusMessage.setNominalExecutionDuration(getTrajectoryDuration());
      executionStatusMessage.setElapsedExecutionTime(executionTimer.getElapsedTime());
      executionStatusMessage.setStartOrientationDistanceToGoal(startOrientationDistanceToGoal);
      executionStatusMessage.setCurrentOrientationDistanceToGoal(completionCalculator.getRotationError());
      executionStatusMessage.setOrientationDistanceToGoalTolerance(ORIENTATION_TOLERANCE);

      if (!isExecuting && wasExecuting && !getHoldPoseInWorldLater())
      {
         disengageHoldPoseInWorld();
      }
   }

   public void disengageHoldPoseInWorld()
   {
      FrameQuaternion frameChestQuaternion = new FrameQuaternion(getConditionalReferenceFrame().get());
      frameChestQuaternion.changeFrame(syncedRobot.getFullRobotModel().getPelvis().getBodyFixedFrame());

      ChestTrajectoryMessage message = new ChestTrajectoryMessage();
      message.getSo3Trajectory()
             .set(HumanoidMessageTools.createSO3TrajectoryMessage(getTrajectoryDuration(),
                                                                  frameChestQuaternion,
                                                                  EuclidCoreTools.zeroVector3D,
                                                                  syncedRobot.getFullRobotModel().getPelvis().getBodyFixedFrame()));
      long frameId = MessageTools.toFrameId(syncedRobot.getFullRobotModel().getPelvis().getBodyFixedFrame());
      message.getSo3Trajectory().getFrameInformation().setDataReferenceFrameId(frameId);

      ros2ControllerHelper.publishToController(message);
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
