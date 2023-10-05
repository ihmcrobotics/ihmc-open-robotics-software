package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ActionExecutionStatusMessage;
import controller_msgs.msg.dds.ChestTrajectoryMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.BehaviorActionExecutor;
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

public class ChestOrientationActionExecutor implements BehaviorActionExecutor
{
   public static final double ORIENTATION_TOLERANCE = Math.toRadians(10.0);

   private final ChestOrientationActionState state;
   private final ChestOrientationActionDefinition definition;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final Timer executionTimer = new Timer();
   private boolean isExecuting;
   private final FramePose3D desiredChestPose = new FramePose3D();
   private final FramePose3D syncedChestPose = new FramePose3D();
   private double startOrientationDistanceToGoal;
   private final ActionExecutionStatusMessage executionStatusMessage = new ActionExecutionStatusMessage();
   private final BehaviorActionCompletionCalculator completionCalculator = new BehaviorActionCompletionCalculator();

   public ChestOrientationActionExecutor(ROS2ControllerHelper ros2ControllerHelper, ROS2SyncedRobotModel syncedRobot, ReferenceFrameLibrary referenceFrameLibrary)
   {
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;

      state = new ChestOrientationActionState(referenceFrameLibrary);
      definition = state.getDefinition();
   }

   @Override
   public void update(int nextExecutionIndex, boolean concurrentActionIsNextForExecution)
   {

   }

   @Override
   public void triggerActionExecution()
   {
      FrameQuaternion frameChestQuaternion = new FrameQuaternion(state.getChestFrame().getReferenceFrame());
      frameChestQuaternion.changeFrame(syncedRobot.getReferenceFrames().getPelvisZUpFrame());

      ChestTrajectoryMessage message = new ChestTrajectoryMessage();
      message.getSo3Trajectory()
             .set(HumanoidMessageTools.createSO3TrajectoryMessage(definition.getTrajectoryDuration(),
                                                                  frameChestQuaternion,
                                                                  EuclidCoreTools.zeroVector3D,
                                                                  ReferenceFrame.getWorldFrame()));
      long frameId = MessageTools.toFrameId(ReferenceFrame.getWorldFrame());
      message.getSo3Trajectory().getFrameInformation().setDataReferenceFrameId(frameId);

      ros2ControllerHelper.publishToController(message);
      executionTimer.reset();

      desiredChestPose.setFromReferenceFrame(state.getChestFrame().getReferenceFrame());
      syncedChestPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getChest().getBodyFixedFrame());
      startOrientationDistanceToGoal = syncedChestPose.getRotation().distance(desiredChestPose.getRotation(), true);
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      desiredChestPose.setFromReferenceFrame(state.getChestFrame().getReferenceFrame());
      syncedChestPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getChest().getBodyFixedFrame());

      boolean wasExecuting = isExecuting;
      isExecuting = !completionCalculator.isComplete(desiredChestPose,
                                                     syncedChestPose,
                                                     Double.NaN, ORIENTATION_TOLERANCE,
                                                     definition.getTrajectoryDuration(),
                                                     executionTimer,
                                                     BehaviorActionCompletionComponent.ORIENTATION);

      executionStatusMessage.setActionIndex(state.getActionIndex());
      executionStatusMessage.setNominalExecutionDuration(definition.getTrajectoryDuration());
      executionStatusMessage.setElapsedExecutionTime(executionTimer.getElapsedTime());
      executionStatusMessage.setStartOrientationDistanceToGoal(startOrientationDistanceToGoal);
      executionStatusMessage.setCurrentOrientationDistanceToGoal(completionCalculator.getRotationError());
      executionStatusMessage.setOrientationDistanceToGoalTolerance(ORIENTATION_TOLERANCE);

      if (!isExecuting && wasExecuting && !definition.getHoldPoseInWorldLater())
      {
         disengageHoldPoseInWorld();
      }
   }

   public void disengageHoldPoseInWorld()
   {
      FrameQuaternion frameChestQuaternion = new FrameQuaternion(state.getChestFrame().getReferenceFrame());
      frameChestQuaternion.changeFrame(syncedRobot.getFullRobotModel().getPelvis().getBodyFixedFrame());

      ChestTrajectoryMessage message = new ChestTrajectoryMessage();
      message.getSo3Trajectory()
             .set(HumanoidMessageTools.createSO3TrajectoryMessage(definition.getTrajectoryDuration(),
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

   @Override
   public ChestOrientationActionState getState()
   {
      return state;
   }

   @Override
   public ChestOrientationActionDefinition getDefinition()
   {
      return definition;
   }
}
