package us.ihmc.avatar.testTools.scs2;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import controller_msgs.msg.dds.MessageCollection;
import controller_msgs.msg.dds.ValkyrieHandFingerTrajectoryMessage;
import controller_msgs.msg.dds.WholeBodyStreamingMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.scs2.SCS2AvatarSimulation;
import us.ihmc.avatar.scs2.SCS2AvatarSimulationFactory;
import us.ihmc.avatar.testTools.ScriptedFootstepGenerator;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.ROS2TopicNameTools;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.yoVariables.exceptions.IllegalOperationException;

public class SCS2AvatarTestingSimulationFactory extends SCS2AvatarSimulationFactory
{
   private final ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.INTRAPROCESS, "ihmc_simulation_test_helper");

   @SuppressWarnings("rawtypes")
   private final Map<Class<?>, IHMCROS2Publisher> defaultControllerPublishers = new HashMap<>();

   private DRCRobotModel robotModel;
   private ScriptedFootstepGenerator scriptedFootstepGenerator;

   public SCS2AvatarTestingSimulationFactory(DRCRobotModel robotModel, CommonAvatarEnvironmentInterface environment)
   {
      setRobotModel(robotModel);
      setCommonAvatarEnvrionmentInterface(environment);

      setRealtimeROS2Node(ROS2Tools.createRealtimeROS2Node(PubSubImplementation.INTRAPROCESS, "ihmc_simulation"));

      List<Class<? extends Command<?, ?>>> controllerSupportedCommands = ControllerAPIDefinition.getControllerSupportedCommands();

      for (Class<? extends Command<?, ?>> command : controllerSupportedCommands)
      {
         Class<?> messageClass = ROS2TopicNameTools.newMessageInstance(command).getMessageClass();
         IHMCROS2Publisher<?> defaultPublisher = createPublisherForController(messageClass);
         defaultControllerPublishers.put(messageClass, defaultPublisher);
      }

      defaultControllerPublishers.put(WholeBodyTrajectoryMessage.class, createPublisherForController(WholeBodyTrajectoryMessage.class));
      defaultControllerPublishers.put(WholeBodyStreamingMessage.class, createPublisherForController(WholeBodyStreamingMessage.class));
      defaultControllerPublishers.put(MessageCollection.class, createPublisherForController(MessageCollection.class));

      defaultControllerPublishers.put(ValkyrieHandFingerTrajectoryMessage.class, createPublisherForController(ValkyrieHandFingerTrajectoryMessage.class));
   }

   public SCS2AvatarTestingSimulation createAvatarTestingSimulation()
   {
      SCS2AvatarTestingSimulation avatarTestingSimulation = new SCS2AvatarTestingSimulation(super.createAvatarSimulation());
      avatarTestingSimulation.setROS2Node(ros2Node);
      avatarTestingSimulation.setDefaultControllerPublishers(defaultControllerPublishers);
      return avatarTestingSimulation;
   }

   @Override
   public SCS2AvatarSimulation createAvatarSimulation()
   {
      throw new IllegalOperationException("Invoke createAvatarTestingSimulation() instead");
   }

   @Override
   public void setRobotModel(DRCRobotModel robotModel)
   {
      super.setRobotModel(robotModel);
      this.robotModel = robotModel;
   }

   public ScriptedFootstepGenerator getScriptedFootstepGenerator()
   {
      if (scriptedFootstepGenerator == null)
         scriptedFootstepGenerator = new ScriptedFootstepGenerator(robotModel.createFullRobotModel());
      return scriptedFootstepGenerator;
   }

   public <T> IHMCROS2Publisher<T> createPublisherForController(Class<T> messageType)
   {
      return createPublisher(messageType, ROS2Tools.getControllerInputTopic(robotModel.getSimpleRobotName()));
   }

   public <T> IHMCROS2Publisher<T> createPublisher(Class<T> messageType, ROS2Topic<?> generator)
   {
      return ROS2Tools.createPublisherTypeNamed(ros2Node, messageType, generator);
   }

   public <T> IHMCROS2Publisher<T> createPublisher(Class<T> messageType, String topicName)
   {
      return ROS2Tools.createPublisher(ros2Node, messageType, topicName);
   }

   public <T> void createSubscriberFromController(Class<T> messageType, ObjectConsumer<T> consumer)
   {
      createSubscriber(messageType, ROS2Tools.getControllerOutputTopic(robotModel.getSimpleRobotName()), consumer);
   }

   public <T> void createSubscriber(Class<T> messageType, ROS2Topic<?> generator, ObjectConsumer<T> consumer)
   {
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, messageType, generator, s -> consumer.consumeObject(s.takeNextData()));
   }

   public <T> void createSubscriber(Class<T> messageType, String topicName, ObjectConsumer<T> consumer)
   {
      ROS2Tools.createCallbackSubscription(ros2Node, messageType, topicName, s -> consumer.consumeObject(s.takeNextData()));
   }
}
