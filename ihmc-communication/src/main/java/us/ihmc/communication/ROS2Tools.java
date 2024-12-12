package us.ihmc.communication;

import us.ihmc.commons.thread.Notification;
import us.ihmc.pubsub.TopicDataType;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.ROS2TopicNameTools;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.thread.SwapReference;

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
 *    <li>{@link SystemMonitorAPI}</li>
 *    <li>{@link PerceptionAPI}</li>
 *    <li>{@link SakeHandAPI}</li>
 *    <li>{@link StateEstimatorAPI}</li>
 *    <li>{@link ToolboxAPIs}</li>
 * </ul>
 *
 * This class used to have methods to create publishers and subscribers. Most of those have been
 * moved to the upstream API. Please use {@link ROS2Node} or {@link RealtimeROS2Node} directly
 * instead now to create those. The API has been improved and it no longer throws useless exceptions.
 *
 * There is a default QoS setting is used when we don't specify the QoS. It is now defined in {@link ROS2QosProfile}
 * and changable via the "ROS_DEFAULT_QOS" environment variable.
 */
public final class ROS2Tools
{
   public static final String IHMC_TOPIC_PREFIX = "ihmc";

   public static final ROS2Topic<?> IHMC_ROOT = new ROS2Topic<>().withPrefix(IHMC_TOPIC_PREFIX);

   /**
    * Allocation free callback where the user only has access to the message in the callback.
    * The user should not take up any significant time in the callback to not slow down the ROS 2
    * thread.
    */
   public static <T> void createVolatileCallbackSubscription(ROS2Node ros2Node, ROS2Topic<T> topic, Consumer<T> callback)
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
   public static <T> SwapReference<T> createSwapReferenceSubscription(ROS2Node ros2Node, ROS2Topic<T> topic, Consumer<T> callback)
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
   public static <T> SwapReference<T> createSwapReferenceSubscription(ROS2Node ros2Node, ROS2Topic<T> topic, Notification callback)
   {
      return createSwapReferenceSubscription(ros2Node, topic, message -> callback.set());
   }
}
