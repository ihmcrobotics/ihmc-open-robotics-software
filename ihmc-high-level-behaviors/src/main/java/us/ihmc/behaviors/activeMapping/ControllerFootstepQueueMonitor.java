package us.ihmc.behaviors.activeMapping;

import controller_msgs.msg.dds.FootstepQueueStatusMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.PlanOffsetStatus;
import controller_msgs.msg.dds.QueuedFootstepStatusMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.communication.HumanoidControllerAPI.getTopic;

public class ControllerFootstepQueueMonitor
{
   private int controllerQueueSize = 0;
   private List<QueuedFootstepStatusMessage> controllerQueue;
   private final AtomicReference<FootstepStatusMessage> footstepStatusMessage = new AtomicReference<>(new FootstepStatusMessage());
   private final AtomicReference<PlanOffsetStatus> planOffsetMessage = new AtomicReference<>(new PlanOffsetStatus());

   private final HumanoidReferenceFrames referenceFrames;
   private final ContinuousHikingLogger continuousHikingLogger;
   private boolean footstepStarted;
   private final AtomicBoolean isWalking = new AtomicBoolean(false);

   public ControllerFootstepQueueMonitor(ROS2Node ros2Node,
                                         String simpleRobotName,
                                         HumanoidReferenceFrames referenceFrames,
                                         ContinuousHikingLogger continuousHikingLogger)
   {
      this.referenceFrames = referenceFrames;
      this.continuousHikingLogger = continuousHikingLogger;

      ros2Node.createSubscription(HumanoidControllerAPI.getTopic(FootstepQueueStatusMessage.class, simpleRobotName), (s) -> footstepQueueStatusReceived(s.takeNextData()));
      ros2Node.createSubscription(HumanoidControllerAPI.getTopic(FootstepStatusMessage.class, simpleRobotName), (s) -> footstepStatusReceived(s.takeNextData()));
      ros2Node.createSubscription(getTopic(PlanOffsetStatus.class, simpleRobotName), (s) -> acceptPlanOffsetStatus(s.takeNextData()));
      ros2Node.createSubscription(getTopic(WalkingStatusMessage.class, simpleRobotName), (s) -> acceptWalkingStatusMessage(s.takeNextData()));
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

      footstepStarted = footstepStatusMessage.getFootstepStatus() == FootstepStatusMessage.FOOTSTEP_STATUS_STARTED;

      this.footstepStatusMessage.set(footstepStatusMessage);
   }

   private void acceptWalkingStatusMessage(WalkingStatusMessage message)
   {
      // Declared locally since this represents the absolute state which other threads can access
      isWalking.set(false);
      WalkingStatus walkingStatus = WalkingStatus.fromByte(message.getWalkingStatus());

      if (walkingStatus == WalkingStatus.STARTED || walkingStatus == WalkingStatus.RESUMED)
      {
         isWalking.set(true);
      }
   }

   private void acceptPlanOffsetStatus(PlanOffsetStatus planOffsetMessage)
   {
      this.planOffsetMessage.set(planOffsetMessage);
   }

   public List<QueuedFootstepStatusMessage> getControllerFootstepQueue()
   {
      return controllerQueue;
   }

   public AtomicReference<FootstepStatusMessage> getFootstepStatusMessage()
   {
      return footstepStatusMessage;
   }

   // TODO Polling this in multiple threads may cause issues as the second time its pulled the value will be null.
   // TODO If needed this should change to handle that case correctly.
   public PlanOffsetStatus pollPlanOffsetMessage()
   {
      return planOffsetMessage.getAndSet(null);
   }

   public boolean isFootstepStarted()
   {
      return footstepStarted;
   }

   // TODO Polling this in multiple threads may cause issues as the second time its pulled the value will be null.
   // TODO If needed this should change to handle that case correctly.
   public boolean pollIsWalking()
   {
      return isWalking.getAndSet(false);
   }
}
