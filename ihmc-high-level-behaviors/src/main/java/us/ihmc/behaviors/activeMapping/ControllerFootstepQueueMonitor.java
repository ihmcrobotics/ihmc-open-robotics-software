package us.ihmc.behaviors.activeMapping;

import controller_msgs.msg.dds.FootstepQueueStatusMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.QueuedFootstepStatusMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.communication.HumanoidControllerAPI.getTopic;

public class ControllerFootstepQueueMonitor
{
   private int controllerQueueSize = 0;
   private List<QueuedFootstepStatusMessage> controllerQueue;
   private final AtomicReference<FootstepStatusMessage> footstepStatusMessage = new AtomicReference<>(new FootstepStatusMessage());

   private final HumanoidReferenceFrames referenceFrames;
   private final ContinuousHikingLogger continuousHikingLogger;
   private boolean footstepStarted;
   private final AtomicBoolean walkingStarted = new AtomicBoolean(false);

   public ControllerFootstepQueueMonitor(ROS2Helper ros2Helper,
                                         String simpleRobotName,
                                         HumanoidReferenceFrames referenceFrames,
                                         ContinuousHikingLogger continuousHikingLogger)
   {
      this.referenceFrames = referenceFrames;
      this.continuousHikingLogger = continuousHikingLogger;
      ros2Helper.subscribeViaCallback(HumanoidControllerAPI.getTopic(FootstepQueueStatusMessage.class, simpleRobotName), this::footstepQueueStatusReceived);
      ros2Helper.subscribeViaCallback(HumanoidControllerAPI.getTopic(FootstepStatusMessage.class, simpleRobotName), this::footstepStatusReceived);
      ros2Helper.subscribeViaCallback(getTopic(WalkingStatusMessage.class, simpleRobotName), this::acceptWalkingStatusMessage);
   }

   private void footstepQueueStatusReceived(FootstepQueueStatusMessage footstepQueueStatusMessage)
   {
      controllerQueue = footstepQueueStatusMessage.getQueuedFootstepList();
      if (controllerQueueSize != footstepQueueStatusMessage.getQueuedFootstepList().size())
      {
         String message = String.format("Latest Controller Queue Footstep Size: " + footstepQueueStatusMessage.getQueuedFootstepList().size());
         LogTools.info(message);
         continuousHikingLogger.appendString(message);
      }

      // For the statistics set the that controller queue size before getting the new one
      controllerQueueSize = footstepQueueStatusMessage.getQueuedFootstepList().size();
   }

   private void footstepStatusReceived(FootstepStatusMessage footstepStatusMessage)
   {
      if (footstepStatusMessage.getFootstepStatus() == FootstepStatusMessage.FOOTSTEP_STATUS_COMPLETED)
      {
         double distance = referenceFrames.getSoleFrame(RobotSide.LEFT)
                                          .getTransformToDesiredFrame(referenceFrames.getSoleFrame(RobotSide.RIGHT))
                                          .getTranslation()
                                          .norm();
      }

      if (footstepStatusMessage.getFootstepStatus() == FootstepStatusMessage.FOOTSTEP_STATUS_STARTED)
      {
         footstepStarted = true;
      }
      else
      {
         footstepStarted = false;
      }

      this.footstepStatusMessage.set(footstepStatusMessage);
   }

   private void acceptWalkingStatusMessage(WalkingStatusMessage message)
   {
      // Declared locally since this represents the absolute state which other threads can access
      walkingStarted.set(false);
      WalkingStatus walkingStatus = WalkingStatus.fromByte(message.getWalkingStatus());

      if (walkingStatus == WalkingStatus.STARTED || walkingStatus == WalkingStatus.RESUMED)
      {
         walkingStarted.set(true);
      }
      else if (walkingStatus == WalkingStatus.ABORT_REQUESTED)
      {
      }
      else if (walkingStatus == WalkingStatus.PAUSED)
      {
      }
      else
      {
      }
   }

   public List<QueuedFootstepStatusMessage> getControllerFootstepQueue()
   {
      return controllerQueue;
   }

   public AtomicReference<FootstepStatusMessage> getFootstepStatusMessage()
   {
      return footstepStatusMessage;
   }

   public boolean isFootstepStarted()
   {
      return footstepStarted;
   }

   public boolean isWalkingStarted()
   {
      return walkingStarted.getAndSet(false);
   }
}
