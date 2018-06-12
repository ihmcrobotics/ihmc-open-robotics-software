package us.ihmc.humanoidBehaviors.behaviors.behaviorServices;

import java.util.HashMap;
import java.util.Map;

import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior.MessageTopicPair;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.pubsub.TopicDataType;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public abstract class BehaviorService
{
   private final Ros2Node ros2Node;
   private final Map<MessageTopicPair<?>, IHMCROS2Publisher<?>> publishers = new HashMap<>();
   private final YoVariableRegistry registry;

   public BehaviorService(String name, Ros2Node ros2Node)
   {
      this.ros2Node = ros2Node;
      registry = new YoVariableRegistry(name);
   }

   public abstract void run();

   public abstract void pause();

   public abstract void destroy();

   @SuppressWarnings("unchecked")
   public <T> IHMCROS2Publisher<T> createPublisher(TopicDataType<T> pubSubType, String topicName)
   {
      Class<T> messageType = (Class<T>) pubSubType.createData().getClass();
      MessageTopicPair<T> key = new MessageTopicPair<>(messageType, topicName);
      IHMCROS2Publisher<T> publisher = (IHMCROS2Publisher<T>) publishers.get(key);

      if (publisher != null)
         return publisher;

      publisher = ROS2Tools.createPublisher(ros2Node, pubSubType, topicName);
      publishers.put(key, publisher);
      return publisher;
   }

   public <T> void createSubscriber(ConcurrentListeningQueue<T> queue, TopicDataType<T> pubSubType, String topicName)
   {
      ROS2Tools.createCallbackSubscription(ros2Node, pubSubType, topicName, s -> queue.put(s.readNextData()));
   }

   public <T> void createSubscriber(ObjectConsumer<T> consumer, TopicDataType<T> pubSubType, String topicName)
   {
      ROS2Tools.createCallbackSubscription(ros2Node, pubSubType, topicName, s -> consumer.consumeObject(s.readNextData()));
   }

   protected Ros2Node getRos2Node()
   {
      return ros2Node;
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }
}
