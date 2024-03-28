package us.ihmc.communication;

import us.ihmc.commons.thread.Notification;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.TopicDataType;
import us.ihmc.ros2.*;
import us.ihmc.tools.thread.SwapReference;
import us.ihmc.util.PeriodicRealtimeThreadSchedulerFactory;
import us.ihmc.util.PeriodicThreadSchedulerFactory;

import java.util.function.Consumer;

public class ROS2Tools
{
   public static final String IHMC_TOPIC_PREFIX = "ihmc";

   public static final ROS2Topic<?> IHMC_ROOT = new ROS2Topic<>().withPrefix(IHMC_TOPIC_PREFIX);

   /** @deprecated Use {@link ROS2Topic#withTypeName} instead. */
   public static <T> ROS2Topic<T> typeNamedTopic(Class<T> messageType)
   {
      return new ROS2Topic<>().withTypeName(messageType);
   }

   private static final RTPSCommunicationFactory FACTORY = new RTPSCommunicationFactory();

   /**
    * Creates a ROS2 node that shares the same implementation as a real-time node <b>but that should
    * not be run in a real-time environment</b>.
    *
    * @param pubSubImplementation the implementation to use.
    * @param nodeName             the name of the new ROS node.
    * @return the ROS node.
    */
   public static RealtimeROS2Node createRealtimeROS2Node(PubSubImplementation pubSubImplementation, String nodeName)
   {
      return new RealtimeROS2Node(pubSubImplementation, nodeName, FACTORY.getDomainId(), FACTORY.getAddressRestriction());
   }

   /**
    * Creates a ROS2 node that is meant to run in real-time environment if the given
    * {@code periodicThreadSchedulerFactory} is a {@link PeriodicRealtimeThreadSchedulerFactory}.
    *
    * @param pubSubImplementation           the implementation to use.
    * @param periodicThreadSchedulerFactory the factory used to create a periodic thread.
    * @param nodeName                       the name of the new ROS node.
    * @return the ROS node.
    */
   public static RealtimeROS2Node createRealtimeROS2Node(PubSubImplementation pubSubImplementation,
                                                         PeriodicThreadSchedulerFactory periodicThreadSchedulerFactory,
                                                         String nodeName)
   {
      return new RealtimeROS2Node(pubSubImplementation, periodicThreadSchedulerFactory, nodeName, FACTORY.getDomainId(), FACTORY.getAddressRestriction());
   }

   public static ROS2Node createROS2Node(PubSubImplementation pubSubImplementation, String nodeName)
   {
      return new ROS2Node(pubSubImplementation, nodeName, FACTORY.getDomainId(), FACTORY.getAddressRestriction());
   }

   /** @deprecated Use {@link ROS2Topic#withTypeName} or look at other examples how to retrieve the topic in a safer way. */
   public static <T> ROS2Subscription<T> createCallbackSubscriptionTypeNamed(ROS2NodeInterface ros2Node,
                                                                             Class<T> messageType,
                                                                             ROS2Topic<?> topicName,
                                                                             NewMessageListener<T> newMessageListener)
   {
      return createCallbackSubscription(ros2Node, typeNamedTopic(messageType).withTopic(topicName), newMessageListener);
   }

   public static <T> ROS2Subscription<T> createCallbackSubscription(ROS2NodeInterface ros2Node,
                                                                    ROS2Topic<T> topic,
                                                                    NewMessageListener<T> newMessageListener)
   {
      return ros2Node.createSubscription(topic, newMessageListener, newMessageListener::onSubscriptionMatched);
   }

   /**
    * Allocation free callback where the user only has access to the message in the callback.
    * The user should not take up any significant time in the callback to not slow down the ROS 2
    * thread.
    */
   public static <T> void createVolatileCallbackSubscription(ROS2NodeInterface ros2Node, ROS2Topic<T> topic, Consumer<T> callback)
   {
      TopicDataType<T> topicDataType = ROS2TopicNameTools.newMessageTopicDataTypeInstance(topic.getType());
      T data = topicDataType.createData();
      ros2Node.createSubscription(topicDataType, subscriber ->
      {
         if (subscriber.takeNextData(data, null))
         {
            callback.accept(data);
         }
      }, topic.getName(), topic.getQoS());
   }

   /** Use when you only need the latest message and need allocation free. */
   public static <T> SwapReference<T> createSwapReferenceSubscription(ROS2NodeInterface ros2Node, ROS2Topic<T> topic, Notification callback)
   {
      TopicDataType<T> topicDataType = ROS2TopicNameTools.newMessageTopicDataTypeInstance(topic.getType());
      SwapReference<T> swapReference = new SwapReference<>(topicDataType::createData);
      ros2Node.createSubscription(topicDataType, subscriber ->
      {
         if (subscriber.takeNextData(swapReference.getForThreadOne(), null))
         {
            swapReference.swap();
            callback.set();
         }
      }, topic.getName(), topic.getQoS());
      return swapReference;
   }
}
