package us.ihmc.behaviors.activeMapping;

import controller_msgs.msg.dds.FootstepQueueStatusMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.QueuedFootstepStatusMessage;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.log.LogTools;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import static us.ihmc.behaviors.activeMapping.ContinuousPlannerSchedulingTask.statistics;

public class ControllerFootstepQueueMonitor
{
   private int controllerQueueSize = 0;
   private List<QueuedFootstepStatusMessage> controllerQueue;

   private List<Consumer<FootstepStatusMessage>> footstepStatusMessageConsumers = new ArrayList<>();

   public ControllerFootstepQueueMonitor(ROS2Helper ros2Helper,
                                         String simpleRobotName)
   {
      ros2Helper.subscribeViaCallback(HumanoidControllerAPI.getTopic(FootstepQueueStatusMessage.class, simpleRobotName), this::footstepQueueStatusReceived);
      ros2Helper.subscribeViaCallback(HumanoidControllerAPI.getTopic(FootstepStatusMessage.class, simpleRobotName), this::footstepStatusReceived);
   }

   private void footstepQueueStatusReceived(FootstepQueueStatusMessage footstepQueueStatusMessage)
   {
      // Set the that controller queue size before getting the new one
      statistics.setLastFootstepQueueLength(controllerQueueSize);

      controllerQueue = footstepQueueStatusMessage.getQueuedFootstepList();
      if (controllerQueueSize != footstepQueueStatusMessage.getQueuedFootstepList().size())
      {
         String message = String.format("State: [%s]: Controller Queue Footstep Size: " + footstepQueueStatusMessage.getQueuedFootstepList().size());
         LogTools.warn(message);
         statistics.appendString(message);
      }
      controllerQueueSize = footstepQueueStatusMessage.getQueuedFootstepList().size();
   }

   private void footstepStatusReceived(FootstepStatusMessage footstepStatusMessage)
   {
      for (Consumer<FootstepStatusMessage> consumer : footstepStatusMessageConsumers)
         consumer.accept(footstepStatusMessage);
   }

   public List<QueuedFootstepStatusMessage> getControllerFootstepQueue()
   {
      return controllerQueue;
   }

   public void attachFootstepStatusMessageConsumer(Consumer<FootstepStatusMessage> consumer)
   {
      this.footstepStatusMessageConsumers.add(consumer);
   }
}
