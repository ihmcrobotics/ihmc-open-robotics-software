package us.ihmc.communication;

import us.ihmc.ros2.Ros2NodeInterface;

import java.util.concurrent.ConcurrentLinkedQueue;

/**
 * To replace usages of RosQueuedSubscription.
 *
 * @param <T>
 */
public class ROS2InputQueue<T>
{
   private final ConcurrentLinkedQueue<T> queue = new ConcurrentLinkedQueue<>();
   private final ROS2Callback<T> ros2Callback;

   public ROS2InputQueue(Ros2NodeInterface ros2Node, Class<T> messageType, String robotName, ROS2ModuleIdentifier identifier)
   {
      ros2Callback = new ROS2Callback<>(ros2Node, messageType, robotName, identifier, this::messageReceivedCallback);
   }

   private void messageReceivedCallback(T incomingData)
   {
      queue.add(incomingData);
   }

   public ConcurrentLinkedQueue getQueue()
   {
      return queue;
   }

   public void setEnabled(boolean enabled)
   {
      ros2Callback.setEnabled(enabled);
   }

   public void destroy()
   {
      ros2Callback.destroy();
   }
}
