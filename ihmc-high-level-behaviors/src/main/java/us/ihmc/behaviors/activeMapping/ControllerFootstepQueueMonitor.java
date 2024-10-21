package us.ihmc.behaviors.activeMapping;

import controller_msgs.msg.dds.FootstepQueueStatusMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.QueuedFootstepStatusMessage;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.controllerAPI.ControllerAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Topic;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class ControllerFootstepQueueMonitor
{
   private int controllerQueueSize = 0;
   private List<QueuedFootstepStatusMessage> controllerQueue;
   private final AtomicReference<FootstepStatusMessage> footstepStatusMessage = new AtomicReference<>(new FootstepStatusMessage());

   private final HumanoidReferenceFrames referenceFrames;
   private final ContinuousPlannerStatistics statistics;

   public ControllerFootstepQueueMonitor(ROS2Helper ros2Helper,
                                         String simpleRobotName,
                                         HumanoidReferenceFrames referenceFrames,
                                         ContinuousPlannerStatistics statistics)
   {
      this.referenceFrames = referenceFrames;
      this.statistics = statistics;
      ROS2Topic<?> controllerOutputTopic = HumanoidControllerAPI.getOutputTopic(simpleRobotName);
      ros2Helper.subscribeViaCallback(ControllerAPI.getTopic(controllerOutputTopic, FootstepQueueStatusMessage.class), this::footstepQueueStatusReceived);
      ros2Helper.subscribeViaCallback(ControllerAPI.getTopic(controllerOutputTopic, FootstepStatusMessage.class), this::footstepStatusReceived);
   }

   private void footstepQueueStatusReceived(FootstepQueueStatusMessage footstepQueueStatusMessage)
   {
      controllerQueue = footstepQueueStatusMessage.getQueuedFootstepList();
      if (controllerQueueSize != footstepQueueStatusMessage.getQueuedFootstepList().size())
      {
         String message = String.format("Latest Controller Queue Footstep Size: " + footstepQueueStatusMessage.getQueuedFootstepList().size());
         LogTools.info(message);
         statistics.appendString(message);
      }

      // For the statistics set the that controller queue size before getting the new one
      statistics.setLastFootstepQueueLength(controllerQueueSize);
      controllerQueueSize = footstepQueueStatusMessage.getQueuedFootstepList().size();
   }

   private void footstepStatusReceived(FootstepStatusMessage footstepStatusMessage)
   {
      if (footstepStatusMessage.getFootstepStatus() == FootstepStatusMessage.FOOTSTEP_STATUS_STARTED)
      {
         statistics.endStepTime();
         statistics.startStepTime();
      }
      else if (footstepStatusMessage.getFootstepStatus() == FootstepStatusMessage.FOOTSTEP_STATUS_COMPLETED)
      {
         // TODO: Use the transfer time (starting now) to start planning (if WAITING_TO_LAND then plan again)
         statistics.setLastFootstepQueueLength(controllerQueueSize);
         statistics.incrementTotalStepsCompleted();

         double distance = referenceFrames.getSoleFrame(RobotSide.LEFT)
                                          .getTransformToDesiredFrame(referenceFrames.getSoleFrame(RobotSide.RIGHT))
                                          .getTranslation()
                                          .norm();
         statistics.setLastLengthCompleted((float) distance);

         statistics.logToFile(true, false);
      }

      this.footstepStatusMessage.set(footstepStatusMessage);
   }

   public List<QueuedFootstepStatusMessage> getControllerFootstepQueue()
   {
      return controllerQueue;
   }

   public AtomicReference<FootstepStatusMessage> getFootstepStatusMessage()
   {
      return footstepStatusMessage;
   }
}
