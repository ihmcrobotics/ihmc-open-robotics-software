package us.ihmc.communication;

import us.ihmc.commons.exception.ExceptionHandler;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.TopicDataType;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.ros2.*;
import us.ihmc.util.PeriodicNonRealtimeThreadSchedulerFactory;

import java.io.IOException;

public class ROS2Tools
{
   public static ExceptionHandler RUNTIME_EXCEPTION = e -> { throw new RuntimeException(e); };
   public static String NAMESPACE = "/us/ihmc"; // ? no idea what this does

   public static RealtimeRos2Node createRealtimeRos2Node(PubSubImplementation pubSubImplementation, String name, ExceptionHandler exceptionHandler)
   {
      try
      {
         return new RealtimeRos2Node(pubSubImplementation, new PeriodicNonRealtimeThreadSchedulerFactory(), name, NAMESPACE);
      }
      catch (IOException e)
      {
         exceptionHandler.handleException(e);
         return null;
      }
   }

   public static Ros2Node createRos2Node(PubSubImplementation pubSubImplementation, String name, ExceptionHandler exceptionHandler)
   {
      try
      {
         return new Ros2Node(pubSubImplementation, name, NAMESPACE);
      }
      catch (IOException e)
      {
         exceptionHandler.handleException(e);
         return null;
      }
   }


   public static <T> void createCallbackSubscription(RealtimeRos2Node realtimeRos2Node, TopicDataType<T> topicDataType, String topicName,
                                                     NewMessageListener newMessageListener, ExceptionHandler exceptionHandler)
   {
      try
      {
         realtimeRos2Node.createCallbackSubscription(topicDataType, topicName, newMessageListener);
      }
      catch (IOException e)
      {
         exceptionHandler.handleException(e);
      }
   }

   public static <T> RealtimeRos2Publisher<T> createPublisher(RealtimeRos2Node realtimeRos2Node, TopicDataType<T> topicDataType, String topicName,
                                                              ExceptionHandler exceptionHandler)
   {
      try
      {
         return realtimeRos2Node.createPublisher(topicDataType, topicName);
      }
      catch (IOException e)
      {
         exceptionHandler.handleException(e);
         return null;
      }
   }

   public static <T> Ros2Publisher<T> createPublisher(Ros2Node ros2Node, TopicDataType<T> topicDataType, String topicName, ExceptionHandler exceptionHandler)
   {
      try
      {
         return ros2Node.createPublisher(topicDataType, topicName);
      }
      catch (IOException e)
      {
         exceptionHandler.handleException(e);
         return null;
      }
   }

   public static void popMessage(Subscriber subscriber, Object message, ExceptionHandler exceptionHandler)
   {
      try
      {
         subscriber.takeNextData(message, null);
      }
      catch (IOException e)
      {
         exceptionHandler.handleException(e);
      }
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
