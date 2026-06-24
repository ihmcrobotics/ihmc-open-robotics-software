package us.ihmc.communication;

import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.jros2.ROS2Message;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2QoSProfile;
import us.ihmc.jros2.ROS2Subscription;
import us.ihmc.jros2.ROS2Topic;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

/**
 * An atomic reference to the latest received message through an optional filter.
 * <p>
 * TODO: Replace call sites with {@link us.ihmc.jros2.ROS2Subscription#readLatest} on control-thread
 * hot paths and direct {@link us.ihmc.jros2.ROS2Subscription} subscriptions elsewhere, then remove
 * this class.
 */
public class ROS2Input<T extends ROS2Message<T>>
{
   private final AtomicReference<T> atomicReference;
   private final MessageFilter<T> messageFilter;
   private final ROS2Node ros2Node;
   private final ROS2Subscription<T> subscription;
   private final boolean reuseBuffer;
   private boolean hasReceivedFirstMessage = false;
   private final TypedNotification<T> messageNotification = new TypedNotification<>();
   private final List<Consumer<T>> userCallbacks = new ArrayList<>();

   public ROS2Input(ROS2Node ros2Node, ROS2Topic<T> topic)
   {
      this(ros2Node, topic, ROS2Message.createInstance(topic.getType()), message -> true);
   }

   public ROS2Input(ROS2Node ros2Node, Class<T> messageType, ROS2Topic<T> topic)
   {
      this(ros2Node, topic, ROS2Message.createInstance(messageType), message -> true);
   }

   public ROS2Input(ROS2Node ros2Node, ROS2Topic<T> topic, MessageFilter<T> messageFilter)
   {
      this(ros2Node, topic, ROS2Message.createInstance(topic.getType()), messageFilter);
   }

   public ROS2Input(ROS2Node ros2Node, ROS2Topic<T> topic, T initialValue, MessageFilter<T> messageFilter)
   {
      this(ros2Node, topic, initialValue, messageFilter, ROS2QoSProfile.DEFAULT);
   }

   public ROS2Input(ROS2Node ros2Node, ROS2Topic<T> topic, T initialValue, MessageFilter<T> messageFilter, ROS2QoSProfile qosProfile)
   {
      this.ros2Node = ros2Node;
      reuseBuffer = initialValue != null;
      atomicReference = new AtomicReference<>(initialValue);
      this.messageFilter = messageFilter;

      subscription = ros2Node.createSubscription(topic, reader ->
      {
         T incoming = reader.read();
         if (incoming == null || !messageFilter.accept(incoming))
            return;

         if (reuseBuffer)
         {
            initialValue.set(incoming);
            messageReceivedCallback(initialValue);
         }
         else
         {
            atomicReference.set(incoming);
            messageReceivedCallback(incoming);
         }
      }, qosProfile);
   }

   public interface MessageFilter<T extends ROS2Message<T>>
   {
      boolean accept(T message);
   }

   private void messageReceivedCallback(T incomingData)
   {
      messageNotification.set(incomingData);
      for (Consumer<T> userCallback : userCallbacks)
         userCallback.accept(incomingData);
      hasReceivedFirstMessage = true;
   }

   public T getLatest()
   {
      return atomicReference.get();
   }

   public TypedNotification<T> getMessageNotification()
   {
      return messageNotification;
   }

   public boolean hasReceivedFirstMessage()
   {
      return hasReceivedFirstMessage;
   }

   public void addCallback(Consumer<T> messageReceivedCallback)
   {
      userCallbacks.add(messageReceivedCallback);
   }

   public void destroy()
   {
      ros2Node.destroySubscription(subscription);
   }
}
