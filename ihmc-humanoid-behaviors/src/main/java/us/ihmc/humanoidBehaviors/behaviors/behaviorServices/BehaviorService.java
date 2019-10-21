package us.ihmc.humanoidBehaviors.behaviors.behaviorServices;

import java.util.HashMap;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.humanoidBehaviors.IHMCHumanoidBehaviorManager;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior.MessageTopicPair;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public abstract class BehaviorService
{
   private final Ros2Node ros2Node;
   private final Map<MessageTopicPair<?>, IHMCROS2Publisher<?>> publishers = new HashMap<>();
   private final YoVariableRegistry registry;
   protected final String robotName;
   private final MessageTopicNameGenerator controllerSubGenerator, controllerPubGenerator;

   public BehaviorService(String robotName, String name, Ros2Node ros2Node)
   {
      this.robotName = robotName;
      this.ros2Node = ros2Node;
      registry = new YoVariableRegistry(name);
      controllerSubGenerator = ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName);
      controllerPubGenerator = ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
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
      String topicName = controllerSubGenerator.generateTopicName(messageType);
      return createPublisher(messageType, topicName);
   }

   public <T> IHMCROS2Publisher<T> createBehaviorOutputPublisher(Class<T> messageType, String topicName)
   {
      return createPublisher(messageType, IHMCHumanoidBehaviorManager.getBehaviorOutputRosTopicPrefix(robotName) + topicName);
   }

   @SuppressWarnings("unchecked")
   public <T> IHMCROS2Publisher<T> createPublisher(Class<T> messageType, String topicName)
   {
      MessageTopicPair<T> key = new MessageTopicPair<>(messageType, topicName);
      IHMCROS2Publisher<T> publisher = (IHMCROS2Publisher<T>) publishers.get(key);

      if (publisher != null)
         return publisher;

      publisher = ROS2Tools.createPublisher(ros2Node, messageType, topicName);
      publishers.put(key, publisher);
      return publisher;
   }

   public <T> void createSubscriberFromController(Class<T> messageType, ObjectConsumer<T> consumer)
   {
      String topicName = controllerPubGenerator.generateTopicName(messageType);
      createSubscriber(messageType, topicName, consumer);
   }

   public <T> void createSubscriber(Class<T> messageType, MessageTopicNameGenerator topicNameGenerator, ObjectConsumer<T> consumer)
   {
      ROS2Tools.createCallbackSubscription(ros2Node, messageType, topicNameGenerator, s -> consumer.consumeObject(s.takeNextData()));
   }

   public <T> void createSubscriber(Class<T> messageType, String topicName, ObjectConsumer<T> consumer)
   {
      ROS2Tools.createCallbackSubscription(ros2Node, messageType, topicName, s -> consumer.consumeObject(s.takeNextData()));
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
