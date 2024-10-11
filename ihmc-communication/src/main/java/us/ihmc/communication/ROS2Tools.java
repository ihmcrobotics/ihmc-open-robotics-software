package us.ihmc.communication;

import us.ihmc.commons.thread.Notification;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.TopicDataType;
import us.ihmc.ros2.*;
import us.ihmc.tools.thread.SwapReference;
import us.ihmc.util.PeriodicRealtimeThreadSchedulerFactory;
import us.ihmc.util.PeriodicThreadSchedulerFactory;

import java.net.InetAddress;
import java.util.function.Consumer;

/**
 * A central place to find topic definitions so there is less duplication and errors.
 * These API classes contains the major topic definitions:
 * <ul>
 *    <li>{@link ActiveMappingAPI}</li>
 *    <li>{@link AutonomyAPI}</li>
 *    <li>{@link DeprecatedAPIs}</li>
 *    <li>{@link FootstepPlannerAPI}</li>
 *    <li>{@link us.ihmc.communication.controllerAPI.ControllerAPI}</li>
 *    <li>{@link HumanoidControllerAPI}</li>
 *    <li>{@link MissionControlAPI}</li>
 *    <li>{@link PerceptionAPI}</li>
 *    <li>{@link SakeHandAPI}</li>
 *    <li>{@link StateEstimatorAPI}</li>
 *    <li>{@link ToolboxAPIs}</li>
 * </ul>
 *
 * This class used to have methods to create publishers and subscribers. Most of those have been
 * moved to the upstream API. Please use {@link ROS2NodeInterface} or {@link RealtimeROS2Node} directly
 * instead now to create those. The API has been improved and it no longer throws useless exceptions.
 *
 * There is a default QoS setting is used when we don't specify the QoS. It is now defined in {@link ROS2QosProfile}
 * and changable via the "ROS_DEFAULT_QOS" environment variable.
 */
public final class ROS2Tools
{
   public static final String IHMC_TOPIC_PREFIX = "ihmc";

   public static final ROS2Topic<?> IHMC_ROOT = new ROS2Topic<>().withPrefix(IHMC_TOPIC_PREFIX);

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

   /**
    * Creates a ROS2 node that only communicates over the loopback address within the host machine.
    * Other machines will not be able to receive messages published using this node.
    * This is useful when publishing large messages for intra-process communication, as to not overwhelm the network.
    *
    * @param pubSubImplementation the implementation to use.
    * @param nodeName             the name of the new ROS node.
    * @return the loopback ROS node
    */
   public static ROS2Node createLoopbackROS2Node(PubSubImplementation pubSubImplementation, String nodeName)
   {
      return new ROS2Node(pubSubImplementation, nodeName, FACTORY.getDomainId(), InetAddress.getLoopbackAddress());
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
   public static <T> SwapReference<T> createSwapReferenceSubscription(ROS2NodeInterface ros2Node, ROS2Topic<T> topic, Consumer<T> callback)
   {
      TopicDataType<T> topicDataType = ROS2TopicNameTools.newMessageTopicDataTypeInstance(topic.getType());
      SwapReference<T> swapReference = new SwapReference<>(topicDataType::createData);
      ros2Node.createSubscription(topicDataType, subscriber ->
      {
         T messageToPack = swapReference.getForThreadOne();
         if (subscriber.takeNextData(messageToPack, null))
         {
            swapReference.swap();
            callback.accept(messageToPack);
         }
      }, topic.getName(), topic.getQoS());
      return swapReference;
   }

   /** Use when you only need the latest message and need allocation free. */
   public static <T> SwapReference<T> createSwapReferenceSubscription(ROS2NodeInterface ros2Node, ROS2Topic<T> topic, Notification callback)
   {
      return createSwapReferenceSubscription(ros2Node, topic, message -> callback.set());
   }
}
