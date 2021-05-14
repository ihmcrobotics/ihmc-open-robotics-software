package us.ihmc.communication;

import geometry_msgs.msg.dds.PosePubSubType;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.TopicDataType;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.ros2.*;
import us.ihmc.ros2.rosidl.geometry_msgs.msg.dds.Pose3DPubSubTypeImpl;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Callback listener to non-null reception of a message on a ROS 2 topic.
 *
 * @param <T> messageType
 */
public class IHMCROS2Callback<T>
{
   private final Consumer<T> messageCallback;
   private ROS2Subscription<T> subscription;
   private volatile boolean enabled = true;

   public IHMCROS2Callback(ROS2NodeInterface ros2Node, ROS2Topic<T> topicName, Consumer<T> messageCallback)
   {
      this(ros2Node, topicName.getType(), topicName.getName(), messageCallback);
   }

   public IHMCROS2Callback(ROS2NodeInterface ros2Node, ROS2Topic<T> topicName, ROS2QosProfile qosProfile, Consumer<T> messageCallback)
   {
      this(ros2Node, topicName.getType(), topicName.getName(), qosProfile, messageCallback);
   }

   public IHMCROS2Callback(ROS2NodeInterface ros2Node, Class<T> messageType, ROS2Topic topicName, Consumer<T> messageCallback)
   {
      this(ros2Node, messageType, topicName.withTypeName(messageType).toString(), messageCallback);
   }

   public IHMCROS2Callback(ROS2NodeInterface ros2Node, Class<T> messageType, String topicName, Consumer<T> messageCallback)
   {
      this(ros2Node, messageType, topicName, ROS2QosProfile.DEFAULT(), messageCallback);
   }

   public IHMCROS2Callback(ROS2NodeInterface ros2Node, Class<T> messageType, String topicName, ROS2QosProfile qosProfile, Consumer<T> messageCallback)
   {
      this.messageCallback = messageCallback;
      ExceptionTools.handle(() ->
      {
         subscription = ros2Node.createSubscription(newMessageTopicDataTypeInstance(messageType), this::nullOmissionCallback, topicName, qosProfile);
      }, DefaultExceptionHandler.RUNTIME_EXCEPTION);
   }

   private static <T> TopicDataType<T> newMessageTopicDataTypeInstance(Class<T> messageType)
   {
      if (messageType.equals(Pose3D.class))
      {
         PosePubSubType.setImplementation(new Pose3DPubSubTypeImpl());
         return (TopicDataType<T>) new PosePubSubType();
      }

      Method pubSubTypeGetter;

      try
      {
         pubSubTypeGetter = messageType.getDeclaredMethod(ROS2TopicNameTools.pubSubTypeGetterName);
      }
      catch (NoSuchMethodException | SecurityException e)
      {
         throw new RuntimeException("Something went wrong when looking up the method "
                                    + messageType.getSimpleName() + "."
                                    + ROS2TopicNameTools.pubSubTypeGetterName + "()."
                                    + e.getMessage(), e);
      }

      TopicDataType<T> topicDataType;

      try
      {
         topicDataType = (TopicDataType<T>) ((Supplier) pubSubTypeGetter.invoke(ROS2TopicNameTools.newMessageInstance(messageType))).get();
      }
      catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e)
      {
         throw new RuntimeException(
               "Something went wrong when invoking the method " + messageType.getSimpleName() + "." + ROS2TopicNameTools.pubSubTypeGetterName + "().", e);
      }
      return topicDataType;
   }

   private void nullOmissionCallback(Subscriber<T> subscriber)
   {
      if (enabled)
      {
         T incomingData = subscriber.takeNextData();
         if (incomingData != null)
         {
            messageCallback.accept(incomingData);
         }
         else
         {
            LogTools.warn("Received null from takeNextData()");
         }
      }
   }

   public void setEnabled(boolean enabled)
   {
      this.enabled = enabled;
   }

   public void destroy()
   {
      if (subscription != null)
      {
         subscription.remove();
      }
   }
}
