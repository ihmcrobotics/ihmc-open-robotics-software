package us.ihmc.humanoidBehaviors.behaviors.behaviorServices;

import java.util.HashMap;
import java.util.Map;

import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.humanoidBehaviors.IHMCHumanoidBehaviorManager;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.registry.YoRegistry;

public abstract class BehaviorService
{
   private final Ros2Node ros2Node;
   private final Map<ROS2Topic, IHMCROS2Publisher<?>> publishers = new HashMap<>();
   private final YoRegistry registry;
   protected final String robotName;
   private final ROS2Topic controllerInputTopic, controllerOutputTopic;

   public BehaviorService(String robotName, String name, Ros2Node ros2Node)
   {
      this.robotName = robotName;
      this.ros2Node = ros2Node;
      registry = new YoRegistry(name);
      controllerInputTopic = ROS2Tools.getControllerInputTopic(robotName);
      controllerOutputTopic = ROS2Tools.getControllerOutputTopic(robotName);
   }

   public abstract void run();

   public abstract void pause();

   public abstract void destroy();

   public MessagerAPI getBehaviorAPI()
   {
      return null;
   }
   
   public <T> IHMCROS2Publisher<T> createPublisherForController(Class<T> messageType)
   {
      ROS2Topic topicName = controllerInputTopic.withType(messageType);
      return createPublisher(messageType, topicName);
   }

   public <T> IHMCROS2Publisher<T> createBehaviorOutputPublisher(Class<T> messageType, String topicName)
   {
      return createPublisher(messageType, IHMCHumanoidBehaviorManager.getBehaviorOutputRosTopicPrefix(robotName).withSuffix(topicName));
   }

   @SuppressWarnings("unchecked")
   public <T> IHMCROS2Publisher<T> createPublisher(Class<T> messageType, ROS2Topic topicName)
   {
      IHMCROS2Publisher<T> publisher = (IHMCROS2Publisher<T>) publishers.get(topicName);

      if (publisher == null)
      {
         publisher = ROS2Tools.createPublisherTypeNamed(ros2Node, messageType, topicName);
         publishers.put(topicName, publisher);
      }

      return publisher;
   }

   public <T> void createSubscriberFromController(Class<T> messageType, ObjectConsumer<T> consumer)
   {
      ROS2Topic topicName = controllerOutputTopic.withType(messageType);
      createSubscriber(messageType, topicName, consumer);
   }

   public <T> void createSubscriber(Class<T> messageType, ROS2Topic topicName, ObjectConsumer<T> consumer)
   {
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, messageType, topicName, s -> consumer.consumeObject(s.takeNextData()));
   }

   public <T> void createSubscriber(Class<T> messageType, String topicName, ObjectConsumer<T> consumer)
   {
      ROS2Tools.createCallbackSubscription(ros2Node, messageType, topicName, s -> consumer.consumeObject(s.takeNextData()));
   }

   protected Ros2Node getRos2Node()
   {
      return ros2Node;
   }

   public YoRegistry getYoVariableRegistry()
   {
      return registry;
   }
}
