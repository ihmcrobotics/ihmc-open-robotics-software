package us.ihmc.humanoidBehaviors.behaviors.behaviorServices;

import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.humanoidBehaviors.IHMCHumanoidBehaviorManager;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.HashMap;
import java.util.Map;

public abstract class BehaviorService
{
   private final ROS2Node ros2Node;
   private final Map<ROS2Topic, ROS2Publisher<?>> publishers = new HashMap<>();
   private final YoRegistry registry;
   protected final String robotName;
   private final ROS2Topic controllerInputTopic, controllerOutputTopic;

   public BehaviorService(String robotName, String name, ROS2Node ros2Node)
   {
      this.robotName = robotName;
      this.ros2Node = ros2Node;
      registry = new YoRegistry(name);
      controllerInputTopic = HumanoidControllerAPI.getInputTopic(robotName);
      controllerOutputTopic = HumanoidControllerAPI.getOutputTopic(robotName);
   }

   public abstract void run();

   public abstract void pause();

   public abstract void destroy();

   public MessagerAPI getBehaviorAPI()
   {
      return null;
   }
   
   public <T> ROS2Publisher<T> createPublisherForController(Class<T> messageType)
   {
      ROS2Topic topicName = controllerInputTopic.withTypeName(messageType);
      return createPublisher(messageType, topicName);
   }

   public <T> ROS2Publisher<T> createBehaviorOutputPublisher(Class<T> messageType, String topicName)
   {
      return createPublisher(messageType, IHMCHumanoidBehaviorManager.getBehaviorOutputRosTopicPrefix(robotName).withSuffix(topicName));
   }

   @SuppressWarnings("unchecked")
   public <T> ROS2Publisher<T> createPublisher(Class<T> messageType, ROS2Topic topicName)
   {
      ROS2Publisher<T> publisher = (ROS2Publisher<T>) publishers.get(topicName);

      if (publisher == null)
      {
         publisher = ros2Node.createPublisher(topicName.withTypeName(messageType));
         publishers.put(topicName, publisher);
      }

      return publisher;
   }

   public <T> void createSubscriberFromController(Class<T> messageType, ObjectConsumer<T> consumer)
   {
      ROS2Topic topicName = controllerOutputTopic.withTypeName(messageType);
      createSubscriber(messageType, topicName, consumer);
   }

   public <T> void createSubscriber(Class<T> messageType, ROS2Topic topicName, ObjectConsumer<T> consumer)
   {
      ros2Node.createSubscription(((ROS2Topic<?>) topicName).withTypeName(messageType), s -> consumer.consumeObject(s.takeNextData()));
   }

   protected ROS2Node getROS2Node()
   {
      return ros2Node;
   }

   public YoRegistry getYoVariableRegistry()
   {
      return registry;
   }
}
