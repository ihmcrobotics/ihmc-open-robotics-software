package us.ihmc.communication;

import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.controllerAPI.ControllerAPI;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2QoSProfile;
import us.ihmc.jros2.ROS2Subscription;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.jros2.ROS2Message;
import us.ihmc.jros2.ROS2MessageReader;
import us.ihmc.jros2.AsyncROS2Node;
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
 * moved to the upstream API. Please use {@link ROS2Node} or {@link AsyncROS2Node} directly
 * instead now to create those. The API has been improved and it no longer throws useless exceptions.
 */
public final class ROS2Tools
{
   public static final String IHMC_TOPIC_PREFIX = "ihmc";

   public static final HumanoidROS2Topic<?> IHMC_ROOT = new HumanoidROS2Topic<>().withPrefix(IHMC_TOPIC_PREFIX);

   /** Invokes the callback only when {@link ROS2MessageReader#read()} returns a non-null message. */
   public static <T extends ROS2Message<T>> void readIfPresent(ROS2MessageReader<T> reader, Consumer<T> callback)
   {
      T message = reader.read();
      if (message != null)
         callback.accept(message);
   }

   /**
    * Returns the QoS profile for a topic, using {@link ControllerAPI#getQoS(Class)} when the topic has a message type.
    */
   public static ROS2QoSProfile getTopicQoS(ROS2Topic<?> topic)
   {
      if (topic != null && topic.getType() != null)
         return ControllerAPI.getQoS(topic.getType());

      return ROS2QoSProfile.DEFAULT;
   }

   /**
    * Creates a subscription using the QoS profile appropriate for the topic's message type.
    */
   public static <T extends ROS2Message<T>> ROS2Subscription<T> createSubscription(ROS2Node ros2Node,
                                                                                   ROS2Topic<T> topic,
                                                                                   Consumer<T> callback)
   {
      return ros2Node.createSubscription(topic, reader -> readIfPresent(reader, callback), getTopicQoS(topic));
   }

   /**
    * Volatile callback where the user only has access to the message in the callback.
    * The message is only valid within the callback.
    */
   public static <T extends ROS2Message<T>> void createVolatileCallbackSubscription(ROS2Node ros2Node, ROS2Topic<T> topic, Consumer<T> callback)
   {
      createSubscription(ros2Node, topic, callback);
   }

   /** Use when you only need the latest message with swap reference pattern. */
   public static <T extends ROS2Message<T>> SwapReference<T> createSwapReferenceSubscription(ROS2Node ros2Node, ROS2Topic<T> topic, Consumer<T> callback)
   {
      SwapReference<T> swapReference = new SwapReference<>(() -> ROS2Message.createInstance(topic.getType()));

      createSubscription(ros2Node, topic, readMessage ->
      {
         T messageToPack = swapReference.getForThreadOne();
         messageToPack.set(readMessage);
         swapReference.swap();
         callback.accept(messageToPack);
      });
      return swapReference;
   }

   /** Use when you only need the latest message and need allocation free. */
   public static <T extends ROS2Message<T>> SwapReference<T> createSwapReferenceSubscription(ROS2Node ros2Node, ROS2Topic<T> topic, Notification callback)
   {
      return createSwapReferenceSubscription(ros2Node, topic, message -> callback.set());
   }

   public static <T extends ROS2Message<T>> TypedNotification<T> createNotificationSubscription(ROS2Node ros2Node, ROS2Topic<T> topic)
   {
      TypedNotification<T> typedNotification = new TypedNotification<>();
      createSubscription(ros2Node, topic, typedNotification::set);
      return typedNotification;
   }
}
