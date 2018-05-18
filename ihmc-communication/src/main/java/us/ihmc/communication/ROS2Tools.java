package us.ihmc.communication;

import java.io.IOException;

import us.ihmc.commons.exception.ExceptionHandler;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.TopicDataType;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.ros2.NewMessageListener;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.ros2.Ros2QosProfile;
import us.ihmc.util.PeriodicNonRealtimeThreadSchedulerFactory;

public class ROS2Tools
{
   public static ExceptionHandler RUNTIME_EXCEPTION = e -> {
      throw new RuntimeException(e);
   };
   public static String NAMESPACE = "/us/ihmc"; // ? no idea what this does

   public static RealtimeRos2Node createRealtimeRos2Node(PubSubImplementation pubSubImplementation, String topicName, ExceptionHandler exceptionHandler)
   {
      try
      {
         return new RealtimeRos2Node(pubSubImplementation, new PeriodicNonRealtimeThreadSchedulerFactory(), topicName, NAMESPACE);
      }
      catch (IOException e)
      {
         exceptionHandler.handleException(e);
         return null;
      }
   }

   public static <T> void createCallbackSubscription(PubSubImplementation pubSubImplementation, RealtimeRos2Node realtimeRos2Node,
                                                     TopicDataType<T> topicDataType, String topicName, NewMessageListener newMessageListener,
                                                     ExceptionHandler exceptionHandler)
   {
      try
      {
         realtimeRos2Node.createCallbackSubscription(topicDataType, "/" + topicName, newMessageListener, Ros2QosProfile.DEFAULT());
      }
      catch (IOException e)
      {
         exceptionHandler.handleException(e);
      }
   }

   public static <T> ROS2ObjectPublisher<T> createPublisher(PubSubImplementation pubSubImplementation, RealtimeRos2Node realtimeRos2Node,
                                                            TopicDataType<T> topicDataType, String topicName, ExceptionHandler exceptionHandler)
   {
      try
      {
         return new ROS2ObjectPublisher<T>(realtimeRos2Node.createPublisher(topicDataType, "/" + topicName, Ros2QosProfile.DEFAULT(), 10));
      }
      catch (IOException e)
      {
         exceptionHandler.handleException(e);
         return null;
      }
   }

   public static void popMessage(Subscriber subscriber, Object message, SampleInfo sampleInfo)
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
}
