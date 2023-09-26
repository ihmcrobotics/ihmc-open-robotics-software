package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ActionExecutionStatusMessage;
import behavior_msgs.msg.dds.BodyPartPoseStatusMessage;
import controller_msgs.msg.dds.ChestTrajectoryMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.BehaviorAction;
import us.ihmc.behaviors.sequence.BehaviorActionCompletionCalculator;
import us.ihmc.behaviors.sequence.BehaviorActionCompletionComponent;
import us.ihmc.behaviors.sequence.BehaviorActionSequence;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.Timer;

public class ChestOrientationAction extends ChestOrientationActionData implements BehaviorAction
{
   public static final double ORIENTATION_TOLERANCE = Math.toRadians(10.0);

   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private int actionIndex;
   private final BodyPartPoseStatusMessage chestPoseStatus = new BodyPartPoseStatusMessage();
   private final IHMCROS2Input<BodyPartPoseStatusMessage> pelvisPositionStatusSubscription;
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
      setReferenceFrameLibrary(referenceFrameLibrary);

      pelvisPositionStatusSubscription = ros2ControllerHelper.subscribe(BehaviorActionSequence.PELVIS_POSITION_STATUS);
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
         if (pelvisPositionStatusSubscription.getMessageNotification().poll())
         {
            BodyPartPoseStatusMessage pelvisPositionStatusMessage = pelvisPositionStatusSubscription.getLatest();
            ModifiableReferenceFrame pelvisInteractableReferenceFrame = new ModifiableReferenceFrame(getReferenceFrameLibrary().findFrameByName(pelvisPositionStatusMessage.getParentFrame()
                                                                                                                              .getString(0)).get());
            pelvisInteractableReferenceFrame.update(transformToParent -> MessageTools.toEuclid(pelvisPositionStatusMessage.getTransformToParent(), transformToParent));
            pelvisInteractableReferenceFrame.changeParentFrameWithoutMoving(ReferenceFrame.getWorldFrame());

            if (previousPelvisHeightInWorld < 0.0)
               heightVariationInWorld = 0.0;
            else
            {
               heightVariationInWorld = pelvisInteractableReferenceFrame.getTransformToParent().getTranslationZ() - previousPelvisHeightInWorld;
               LogTools.info("{} - {} = {}", pelvisInteractableReferenceFrame.getTransformToParent().getTranslationZ(), previousPelvisHeightInWorld, heightVariationInWorld);
            }
            previousPelvisHeightInWorld = pelvisInteractableReferenceFrame.getTransformToParent().getTranslationZ();
            LogTools.info(previousPelvisHeightInWorld);
         }

         if (heightVariationInWorld > 0.0)
         {
            FramePose3D updatedChestFramePose = new FramePose3D(getParentFrame(), getTransformToParent());
            updatedChestFramePose.changeFrame(ReferenceFrame.getWorldFrame());
            updatedChestFramePose.getTranslation().setZ(updatedChestFramePose.getTranslationZ() + heightVariationInWorld);

            chestPoseStatus.getParentFrame().add(ReferenceFrame.getWorldFrame().getName());
            MessageTools.toMessage(new RigidBodyTransform(updatedChestFramePose.getOrientation(), updatedChestFramePose.getPosition()),
                                   chestPoseStatus.getTransformToParent());
         }
         else
         {
            chestPoseStatus.getParentFrame().resetQuick();
            chestPoseStatus.getParentFrame().add(getParentFrame().getName());
            MessageTools.toMessage(getTransformToParent(), chestPoseStatus.getTransformToParent());
         }
         // send an update of the pose of the chest. Arms IK will be computed wrt this chest pose
         ros2ControllerHelper.publish(BehaviorActionSequence.CHEST_POSE_STATUS, chestPoseStatus);
      }
   }

   @Override
   public void triggerActionExecution()
   {
      FrameQuaternion frameChestQuaternion = new FrameQuaternion(getReferenceFrame());
      frameChestQuaternion.changeFrame(ReferenceFrame.getWorldFrame());

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

      desiredChestPose.setFromReferenceFrame(getReferenceFrame());
      syncedChestPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getChest().getBodyFixedFrame());
      startOrientationDistanceToGoal = syncedChestPose.getRotation().distance(desiredChestPose.getRotation(), true);
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      desiredChestPose.setFromReferenceFrame(getReferenceFrame());
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
      FrameQuaternion frameChestQuaternion = new FrameQuaternion(getReferenceFrame());
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
