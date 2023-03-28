package us.ihmc.avatar.testTools.scs2;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import ihmc_common_msgs.msg.dds.MessageCollection;
import controller_msgs.msg.dds.ValkyrieHandFingerTrajectoryMessage;
import controller_msgs.msg.dds.WholeBodyStreamingMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.scs2.SCS2AvatarSimulation;
import us.ihmc.avatar.scs2.SCS2AvatarSimulationFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.StepGeneratorAPIDefinition;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.ROS2TopicNameTools;
import us.ihmc.scs2.session.Session;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.yoVariables.exceptions.IllegalOperationException;

public class SCS2AvatarTestingSimulationFactory extends SCS2AvatarSimulationFactory
{
   private final OptionalFactoryField<Boolean> createVideo = new OptionalFactoryField<>("createVideo", false);
   private final OptionalFactoryField<Boolean> keepSCSUp = new OptionalFactoryField<>("keepSCSUp", false);

   private final PubSubImplementation pubSubImplementation = PubSubImplementation.INTRAPROCESS;
   private final ROS2Node ros2Node = ROS2Tools.createROS2Node(pubSubImplementation, "ihmc_simulation_test_helper");

   @SuppressWarnings("rawtypes")
   private final Map<Class<?>, IHMCROS2Publisher> defaultControllerPublishers = new HashMap<>();

   public static SCS2AvatarTestingSimulation createDefaultTestSimulation(DRCRobotModel robotModel, SimulationTestingParameters simulationTestingParameters)
   {
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = createDefaultTestSimulationFactory(robotModel, simulationTestingParameters);
      return simulationTestHelperFactory.createAvatarTestingSimulation();
   }

   public static SCS2AvatarTestingSimulation createDefaultTestSimulation(DRCRobotModel robotModel,
                                                                         CommonAvatarEnvironmentInterface environment,
                                                                         SimulationTestingParameters simulationTestingParameters)
   {
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = createDefaultTestSimulationFactory(robotModel, environment, simulationTestingParameters);
      return simulationTestHelperFactory.createAvatarTestingSimulation();
   }

   public static SCS2AvatarTestingSimulationFactory createDefaultTestSimulationFactory(DRCRobotModel robotModel,
                                                                                       SimulationTestingParameters simulationTestingParameters)
   {
      return createDefaultTestSimulationFactory(robotModel, null, simulationTestingParameters);
   }

   public static SCS2AvatarTestingSimulationFactory createDefaultTestSimulationFactory(DRCRobotModel robotModel,
                                                                                       CommonAvatarEnvironmentInterface environment,
                                                                                       SimulationTestingParameters simulationTestingParameters)
   {
      if (environment == null)
         environment = new DefaultCommonAvatarEnvironment();
      SCS2AvatarTestingSimulationFactory simulationFactory = new SCS2AvatarTestingSimulationFactory(robotModel, environment);
      simulationFactory.setDefaultHighLevelHumanoidControllerFactory();
      simulationFactory.setup(simulationTestingParameters);
      return simulationFactory;
   }

   public SCS2AvatarTestingSimulationFactory(DRCRobotModel robotModel, CommonAvatarEnvironmentInterface environment)
   {
      setRobotModel(robotModel);
      setCommonAvatarEnvrionmentInterface(environment);

      setRealtimeROS2Node(ROS2Tools.createRealtimeROS2Node(pubSubImplementation, "ihmc_simulation"));

      List<Class<? extends Command<?, ?>>> controllerSupportedCommands = ControllerAPIDefinition.getControllerSupportedCommands();

      for (Class<? extends Command<?, ?>> command : controllerSupportedCommands)
      {
         Class<?> messageClass = ROS2TopicNameTools.newMessageInstance(command).getMessageClass();
         IHMCROS2Publisher<?> defaultPublisher = createPublisherForController(messageClass);
         defaultControllerPublishers.put(messageClass, defaultPublisher);
      }

      List<Class<? extends Command<?, ?>>> stepGeneratorSupportedCommands = StepGeneratorAPIDefinition.getStepGeneratorSupportedCommands();

      for (Class<? extends Command<?, ?>> command : stepGeneratorSupportedCommands)
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
      return createAvatarTestingSimulation(null);
   }

   public SCS2AvatarTestingSimulation createAvatarTestingSimulation(String simulationName)
   {
      // Access the factory fields before they get disposed of.
      boolean createVideo = this.createVideo.get();
      boolean keepSCSUp = this.keepSCSUp.get();

      setSimulationName(simulationName != null ? simulationName : Session.retrieveCallingTestName());
      SCS2AvatarTestingSimulation avatarTestingSimulation = new SCS2AvatarTestingSimulation(super.createAvatarSimulation());
      avatarTestingSimulation.setROS2Node(ros2Node);
      avatarTestingSimulation.setDefaultControllerPublishers(defaultControllerPublishers);
      avatarTestingSimulation.setCreateVideo(createVideo);
      avatarTestingSimulation.setKeepSCSUp(keepSCSUp);
      // TODO This guy needs to be created before the robot is completely standing. Should cleanup QueuedControllerCommandGenerator, quite a mess.
      avatarTestingSimulation.getQueuedControllerCommands();
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
   }

   public void setup(SimulationTestingParameters parameters)
   {
      setShowGUI(parameters.getCreateGUI());
      setUsePerfectSensors(parameters.getUsePefectSensors());
      setRunMultiThreaded(parameters.getRunMultiThreaded());
      setSimulationDataBufferSize(parameters.getDataBufferSize());
      setCreateVideo(parameters.getCreateSCSVideos());
      setKeepSCSUp(parameters.getKeepSCSUp());
   }

   public void setCreateVideo(boolean createVideo)
   {
      this.createVideo.set(createVideo);
   }

   public void setKeepSCSUp(boolean keepSCSUp)
   {
      this.keepSCSUp.set(keepSCSUp);
   }

   public <T> IHMCROS2Publisher<T> createPublisherForController(Class<T> messageType)
   {
      return createPublisher(messageType, ROS2Tools.getControllerInputTopic(robotModel.get().getSimpleRobotName()));
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
      createSubscriber(messageType, ROS2Tools.getControllerOutputTopic(robotModel.get().getSimpleRobotName()), consumer);
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
