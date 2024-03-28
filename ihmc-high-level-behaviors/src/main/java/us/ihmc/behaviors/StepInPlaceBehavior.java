package us.ihmc.behaviors;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import std_msgs.msg.dds.Bool;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.behaviors.behaviorTree.LocalOnlyBehaviorTreeNodeExecutor;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.OldBehaviorAPI;
import us.ihmc.ros2.ROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.RemoteHumanoidRobotInterface;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.Destroyable;
import us.ihmc.tools.thread.PausablePeriodicThread;

/**
 * A simple example behavior.
 */
public class StepInPlaceBehavior extends LocalOnlyBehaviorTreeNodeExecutor implements Destroyable
{
   private final BehaviorHelper helper;

   private final ROS2Input<Bool> stepping;
   private final AtomicInteger footstepsTaken = new AtomicInteger(2);
   private final AtomicLong lastFootstepTakenID = new AtomicLong(0);
   private final AtomicLong footstepID = new AtomicLong();
   private final PausablePeriodicThread mainThread;
   private final RemoteHumanoidRobotInterface robotInterface;
   private final ROS2SyncedRobotModel syncedRobot;

   public StepInPlaceBehavior(BehaviorHelper helper)
   {
      LogTools.debug("Initializing step in place behavior");

      this.helper = helper;
      robotInterface = helper.getOrCreateRobotInterface();
      syncedRobot = robotInterface.newSyncedRobot();

      robotInterface.createFootstepStatusCallback(this::consumeFootstepStatus);
      stepping = helper.subscribe(API.STEPPING);
      helper.subscribeViaCallback(API.ABORT, message -> doOnAbort(message.getData()));

      mainThread = helper.createPausablePeriodicThread(getClass(), 1.0, this::stepInPlace);
   }

   @Override
   public BehaviorTreeNodeStatus determineStatus()
   {
      return BehaviorTreeNodeStatus.SUCCESS;
   }

   public void setEnabled(boolean enabled)
   {
      LogTools.info("Step in place behavior selected = {}", enabled);

      mainThread.setRunning(enabled);
   }

   private void doOnAbort(boolean abort)
   {
      if (abort)
      {
         LogTools.info("Abort received. Shutting down threadScheduler.");
         mainThread.stop();
      }
   }

   private void consumeFootstepStatus(FootstepStatusMessage footstepStatusMessage)
   {
      LogTools.info("consumeFootstepStatus: " + footstepStatusMessage);

      if (footstepStatusMessage.getFootstepStatus() == FootstepStatus.COMPLETED.toByte())
      {
         int footstepsTakenSoFar = footstepsTaken.incrementAndGet();
         lastFootstepTakenID.set(footstepStatusMessage.getSequenceId());
         LogTools.info("Have taken " + footstepsTakenSoFar + " footsteps. Last one had id: " + lastFootstepTakenID.get());
      }
   }

   private void stepInPlace()
   {
      boolean wasStepping = stepping.hasReceivedFirstMessage() && stepping.getLatest().getData();
      if (stepping.getMessageNotification().poll())
      {
         if (!wasStepping)
         {
            LogTools.info("Starting to step");
         }

         //         if (!behaviorHelper.isRobotWalking())
         if (footstepsTaken.compareAndSet(2, 0))
         {
            LogTools.info("Sending steps");

            syncedRobot.update();
            FootstepDataListMessage footstepList = createTwoStepInPlaceSteps(syncedRobot.getFullRobotModel());
            robotInterface.requestWalk(footstepList);
         }
      }
      else if (wasStepping)
      {
         LogTools.info("Stopped stepping");
      }
   }

   private FootstepDataListMessage createTwoStepInPlaceSteps(FullHumanoidRobotModel fullRobotModel)
   {
      FootstepDataListMessage footstepList = new FootstepDataListMessage();
      RecyclingArrayList<FootstepDataMessage> footstepDataMessages = footstepList.getFootstepDataList();

      for (RobotSide side : RobotSide.values)
      {
         MovingReferenceFrame stepFrame = fullRobotModel.getSoleFrame(side);
         FramePoint3D footLocation = new FramePoint3D(stepFrame);
         FrameQuaternion footOrientation = new FrameQuaternion(stepFrame);
         footLocation.changeFrame(ReferenceFrame.getWorldFrame());
         footOrientation.changeFrame(ReferenceFrame.getWorldFrame());

         FootstepDataMessage footstepDataMessage = HumanoidMessageTools.createFootstepDataMessage(side, footLocation, footOrientation);
         footstepDataMessage.setSequenceId(footstepID.incrementAndGet());
         footstepDataMessages.add().set(footstepDataMessage);
      }
      footstepList.setAreFootstepsAdjustable(true);
      return footstepList;
   }

   public static class API
   {
      private static final String MODULE_NAME = OldBehaviorAPI.BEHAVIOR_MODULE_NAME + "/step_in_place";
      private static final ROS2Topic<?> BASE_TOPIC = ROS2Tools.IHMC_ROOT.withModule(MODULE_NAME);

      public static final ROS2Topic<Bool> STEPPING = BASE_TOPIC.withType(Bool.class).withSuffix("stepping");
      public static final ROS2Topic<Bool> ABORT = BASE_TOPIC.withType(Bool.class).withSuffix("abort");
   }
}
