package us.ihmc.communication;

import us.ihmc.ros2.Ros2Node;

import java.util.concurrent.atomic.AtomicReference;

/**
 * An atomic reference to the latest received message through an optional filter.
 *
 * @param <T> messageType
 */
public class ROS2Input<T>
{
   private final AtomicReference<T> atomicReference;
   private final MessageFilter<T> messageFilter;

   public ROS2Input(Ros2Node ros2Node, Class<T> messageType, String robotName, String moduleName)
   {
      this(ros2Node, messageType, robotName, moduleName, message -> true);
   }

   public ROS2Input(Ros2Node ros2Node, Class<T> messageType, String robotName, String moduleName, MessageFilter<T> messageFilter)
   {
      atomicReference = new AtomicReference<>(ROS2Tools.newMessageInstance(messageType));
      this.messageFilter = messageFilter;
      new ROS2Callback<>(ros2Node, messageType, robotName, moduleName, this::messageReceivedCallback);
   }

   public interface MessageFilter<T>
   {
      boolean accept(T message);
   }

   private void messageReceivedCallback(T incomingData)
   {
      if (messageFilter.accept(incomingData))
      {
         atomicReference.set(incomingData);
      }
   }

   public T getLatest()
   {
      return atomicReference.get();
   }
}
