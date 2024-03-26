package us.ihmc.communication;

import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.ros2.ROS2Callback;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.ROS2TopicNameTools;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

/**
 * An atomic reference to the latest received message through an optional filter.
 *
 * @param <T> messageType
 */
public class IHMCROS2Input<T>
{
   private final AtomicReference<T> atomicReference;
   private final MessageFilter<T> messageFilter;
   private boolean hasReceivedFirstMessage = false;
   private final ROS2Callback<T> ros2Callback;
   private final TypedNotification<T> messageNotification = new TypedNotification<>();
   private final List<Consumer<T>> userCallbacks = new ArrayList<>();

   public IHMCROS2Input(ROS2NodeInterface ros2Node, ROS2Topic<T> topic)
   {
      this(ros2Node, topic, ROS2TopicNameTools.newMessageInstance(topic.getType()), message -> true);
   }

   public IHMCROS2Input(ROS2NodeInterface ros2Node, Class<T> messageType, ROS2Topic<?> topic)
   {
      this(ros2Node, messageType, topic.getName(), topic.getQoS());
   }

   public IHMCROS2Input(ROS2NodeInterface ros2Node, Class<T> messageType, ROS2Topic<?> topic, MessageFilter<T> messageFilter)
   {
      this(ros2Node, messageType, topic.getName(), topic.getQoS(), ROS2TopicNameTools.newMessageInstance(messageType), messageFilter);
   }

   public IHMCROS2Input(ROS2NodeInterface ros2Node, Class<T> messageType, String topicName, ROS2QosProfile qosProfile)
   {
      this(ros2Node, messageType, topicName, qosProfile, ROS2TopicNameTools.newMessageInstance(messageType), message -> true);
   }

   public IHMCROS2Input(ROS2NodeInterface ros2Node, Class<T> messageType, ROS2Topic<?> topic, T initialValue, MessageFilter<T> messageFilter)
   {
      this(ros2Node, messageType, topic.getName(), topic.getQoS(), initialValue, messageFilter);
   }

   public IHMCROS2Input(ROS2NodeInterface ros2Node, ROS2Topic<T> topic, T initialValue, MessageFilter<T> messageFilter)
   {
      this(ros2Node, topic.getType(), topic.getName(), topic.getQoS(), initialValue, messageFilter);
   }

   public IHMCROS2Input(ROS2NodeInterface ros2Node, Class<T> messageType, String topicName, ROS2QosProfile qosProfile, T initialValue, MessageFilter<T> messageFilter)
   {
      atomicReference = new AtomicReference<>(initialValue);
      this.messageFilter = messageFilter;
      ros2Callback = new ROS2Callback<>(ros2Node, messageType, topicName, qosProfile, this::messageReceivedCallback);
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
         messageNotification.set(incomingData);
         for (Consumer<T> userCallback : userCallbacks)
         {
            userCallback.accept(incomingData);
         }
         hasReceivedFirstMessage = true;
      }
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

   public void setEnabled(boolean enabled)
   {
      ros2Callback.setEnabled(enabled);
   }

   public void addCallback(Consumer<T> messageReceivedCallback)
   {
      userCallbacks.add(messageReceivedCallback);
   }

   public void destroy()
   {
      ros2Callback.destroy();
   }
}
