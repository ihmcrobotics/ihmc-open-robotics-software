package us.ihmc.communication;

import java.io.IOException;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.function.Supplier;

import org.apache.commons.lang3.StringUtils;

import com.google.common.base.CaseFormat;

import us.ihmc.commons.exception.ExceptionHandler;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.TopicDataType;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.ros2.NewMessageListener;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.ros2.Ros2QosProfile;
import us.ihmc.util.PeriodicNonRealtimeThreadSchedulerFactory;

public class ROS2Tools
{
   public final static ExceptionHandler RUNTIME_EXCEPTION = e -> {
      throw new RuntimeException(e);
   };
   public final static String NAMESPACE = "/us/ihmc"; // ? no idea what this does

   public static RealtimeRos2Node createRealtimeRos2Node(PubSubImplementation pubSubImplementation, String nodeName)
   {
      return createRealtimeRos2Node(pubSubImplementation, nodeName, RUNTIME_EXCEPTION);
   }

   public static RealtimeRos2Node createRealtimeRos2Node(PubSubImplementation pubSubImplementation, String nodeName, ExceptionHandler exceptionHandler)
   {
      try
      {
         return new RealtimeRos2Node(pubSubImplementation, new PeriodicNonRealtimeThreadSchedulerFactory(), nodeName, NAMESPACE);
      }
      catch (IOException e)
      {
         exceptionHandler.handleException(e);
         return null;
      }
   }

   public static Ros2Node createRos2Node(PubSubImplementation pubSubImplementation, String nodeName)
   {
      return createRos2Node(pubSubImplementation, nodeName, RUNTIME_EXCEPTION);
   }

   public static Ros2Node createRos2Node(PubSubImplementation pubSubImplementation, String nodeName, ExceptionHandler exceptionHandler)
   {
      try
      {
         return new Ros2Node(pubSubImplementation, nodeName, NAMESPACE);
      }
      catch (IOException e)
      {
         exceptionHandler.handleException(e);
         return null;
      }
   }

   public static <T> void createCallbackSubscription(Ros2Node ros2Node, TopicDataType<T> topicDataType, String topicName,
                                                     NewMessageListener<T> newMessageListener)
   {
      createCallbackSubscription(ros2Node, topicDataType, topicName, newMessageListener, RUNTIME_EXCEPTION);
   }

   public static <T> void createCallbackSubscription(Ros2Node ros2Node, TopicDataType<T> topicDataType, String topicName,
                                                     NewMessageListener<T> newMessageListener, ExceptionHandler exceptionHandler)
   {
      try
      {
         ros2Node.createSubscription(topicDataType, newMessageListener, topicName, Ros2QosProfile.DEFAULT());
      }
      catch (IOException e)
      {
         exceptionHandler.handleException(e);
      }
   }

   public static <T> void createCallbackSubscription(RealtimeRos2Node realtimeRos2Node, TopicDataType<T> topicDataType, String topicName,
                                                     NewMessageListener<T> newMessageListener)
   {
      createCallbackSubscription(realtimeRos2Node, topicDataType, topicName, newMessageListener, RUNTIME_EXCEPTION);
   }

   public static <T> void createCallbackSubscription(RealtimeRos2Node realtimeRos2Node, TopicDataType<T> topicDataType, String topicName,
                                                     NewMessageListener<T> newMessageListener, ExceptionHandler exceptionHandler)
   {
      try
      {
         realtimeRos2Node.createCallbackSubscription(topicDataType, topicName, newMessageListener, Ros2QosProfile.DEFAULT());
      }
      catch (IOException e)
      {
         exceptionHandler.handleException(e);
      }
   }

   public static <T> IHMCRealtimeROS2Publisher<T> createPublisher(RealtimeRos2Node realtimeRos2Node, TopicDataType<T> topicDataType, String topicName)
   {
      return createPublisher(realtimeRos2Node, topicDataType, topicName, RUNTIME_EXCEPTION);
   }

   public static <T> IHMCRealtimeROS2Publisher<T> createPublisher(RealtimeRos2Node realtimeRos2Node, TopicDataType<T> topicDataType, String topicName,
                                                                  ExceptionHandler exceptionHandler)
   {
      try
      {
         return new IHMCRealtimeROS2Publisher<T>(realtimeRos2Node.createPublisher(topicDataType, topicName, Ros2QosProfile.DEFAULT(), 10));
      }
      catch (IOException e)
      {
         exceptionHandler.handleException(e);
         return null;
      }
   }

   public static <T> IHMCROS2Publisher<T> createPublisher(Ros2Node ros2Node, TopicDataType<T> topicDataType, String topicName)
   {
      return createPublisher(ros2Node, topicDataType, topicName, RUNTIME_EXCEPTION);
   }

   public static <T> IHMCROS2Publisher<T> createPublisher(Ros2Node ros2Node, TopicDataType<T> topicDataType, String topicName,
                                                          ExceptionHandler exceptionHandler)
   {
      try
      {
         return new IHMCROS2Publisher<T>(ros2Node.createPublisher(topicDataType, topicName, Ros2QosProfile.DEFAULT()));
      }
      catch (IOException e)
      {
         exceptionHandler.handleException(e);
         return null;
      }
   }

   public static <T> void popMessage(Subscriber<T> subscriber, T message, SampleInfo sampleInfo)
   {
      subscriber.takeNextData(message, sampleInfo);
   }

   public static <T> T createMessage(Class<T> messageType, ExceptionHandler exceptionHandler)
   {
      try
      {
         return messageType.newInstance();
      }
      catch (InstantiationException | IllegalAccessException e)
      {
         exceptionHandler.handleException(e);
         return null;
      }
   }

   public static <T extends Settable<T>> String generateDefaultTopicName(Class<T> messageClass)
   {
      return appendTypeToTopicName("/ihmc", messageClass);
   }

   public static String appendTypeToTopicName(String prefix, Class<? extends Settable<?>> messageClass)
   {
      String topicName = messageClass.getSimpleName();
      topicName = StringUtils.removeEnd(topicName, "Packet"); // This makes BehaviorControlModePacket => BehaviorControlMode
      topicName = StringUtils.removeEnd(topicName, "Message"); // This makes ArmTrajectoryMessage => ArmTrajectory
      topicName = "/" + CaseFormat.UPPER_CAMEL.to(CaseFormat.LOWER_UNDERSCORE, topicName); // This makes ArmTrajectory => arm_trajectory
      return topicName;
   }

   public static final String pubSubTypeGetterName = "getPubSubType";

   public static <T> T newMessageInstance(Class<T> messageType)
   {
      try
      {
         return messageType.newInstance();
      }
      catch (InstantiationException | IllegalAccessException e)
      {
         throw new RuntimeException("Something went wrong when invoking " + messageType.getSimpleName() + "'s empty constructor.", e);
      }
   }

   @SuppressWarnings({"unchecked", "rawtypes"})
   public static <T> TopicDataType<T> newMessageTopicDataTypeInstance(Class<T> messageType)
   {
      Method pubSubTypeGetter;

      try
      {
         pubSubTypeGetter = messageType.getDeclaredMethod(pubSubTypeGetterName);
      }
      catch (NoSuchMethodException | SecurityException e)
      {
         throw new RuntimeException("Something went wrong when looking up for the method " + messageType.getSimpleName() + "." + pubSubTypeGetterName + "().",
                                    e);
      }

      TopicDataType<T> topicDataType;

      try
      {
         topicDataType = (TopicDataType<T>) ((Supplier) pubSubTypeGetter.invoke(newMessageInstance(messageType))).get();
      }
      catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e)
      {
         throw new RuntimeException("Something went wrong when invoking the method " + messageType.getSimpleName() + "." + pubSubTypeGetterName + "().", e);
      }
      return topicDataType;
   }
}
