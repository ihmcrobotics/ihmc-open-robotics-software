package us.ihmc.communication;

import us.ihmc.ros2.Ros2Node;

import java.util.concurrent.ConcurrentLinkedQueue;

/**
 * To replace usages of RosQueuedSubscription.
 *
 * @param <T>
 */
public class ROS2InputQueue<T>
{
   ConcurrentLinkedQueue<T> queue = new ConcurrentLinkedQueue<>();

   public ROS2InputQueue(Ros2Node ros2Node, Class<T> messageType, String robotName, ROS2ModuleIdentifier identifier)
   {
      new ROS2Callback<>(ros2Node, messageType, robotName, identifier, this::messageReceivedCallback);
   }

   private void messageReceivedCallback(T incomingData)
   {
      queue.add(incomingData);
   }

   public ConcurrentLinkedQueue getQueue()
   {
      return queue;
   }
}
