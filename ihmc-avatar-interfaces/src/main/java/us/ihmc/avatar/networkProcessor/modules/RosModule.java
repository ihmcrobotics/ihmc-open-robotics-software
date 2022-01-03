package us.ihmc.avatar.networkProcessor.modules;

import java.io.IOException;
import java.net.URI;
import java.util.Set;

import org.ros.internal.message.Message;
import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;

import controller_msgs.msg.dds.LocalizationPacket;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.ros.IHMCPacketToMsgPublisher;
import us.ihmc.avatar.ros.RobotROSClockCalculator;
import us.ihmc.avatar.ros.RosRobotConfigurationDataPublisher;
import us.ihmc.avatar.ros.RosSCSCameraPublisher;
import us.ihmc.avatar.ros.RosSCSLidarPublisher;
import us.ihmc.avatar.ros.RosTfPublisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.ihmcPerception.IHMCETHRosLocalizationUpdateSubscriber;
import us.ihmc.ihmcPerception.RosLocalizationServiceClient;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.FullRobotModelFactory;
import us.ihmc.robotics.partNames.JointNameMap;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.sensorProcessing.parameters.AvatarRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.AvatarRobotRosVisionSensorInformation;
import us.ihmc.sensorProcessing.parameters.AvatarRobotVisionSensorInformation;
import us.ihmc.sensorProcessing.parameters.HumanoidForceSensorInformation;
import us.ihmc.tools.thread.CloseableAndDisposable;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.msgToPacket.converter.GenericROSTranslationTools;

public class RosModule implements CloseableAndDisposable
{
   private static final boolean CREATE_ROS_ECHO_PUBLISHER = false;

   private static final String ROS_NODE_NAME = "networkProcessor/rosModule";

   private final boolean manageROS2Node;
   private final RealtimeROS2Node ros2Node;

   private final RosMainNode rosMainNode;
   private final RobotROSClockCalculator rosClockCalculator;
   private final AvatarRobotRosVisionSensorInformation sensorInformation;
   private final String robotName;
   private RosRobotConfigurationDataPublisher robotConfigurationPublisher;

   public RosModule(DRCRobotModel robotModel, URI rosCoreURI, ObjectCommunicator simulatedSensorCommunicator, RealtimeROS2Node realtimeROS2Node)
   {
      this(robotModel, robotModel.getROSClockCalculator(), robotModel.getSensorInformation(), robotModel.getSensorInformation(), robotModel.getJointMap(),
           rosCoreURI, simulatedSensorCommunicator,
           ROS2Tools.getControllerOutputTopic(robotModel.getRobotDefinition().getName()).withTypeName(RobotConfigurationData.class), realtimeROS2Node);
   }

   public RosModule(DRCRobotModel robotModel, URI rosCoreURI, ObjectCommunicator simulatedSensorCommunicator, PubSubImplementation pubSubImplementation)
   {
      this(robotModel, robotModel.getROSClockCalculator(), robotModel.getSensorInformation(), robotModel.getSensorInformation(), robotModel.getJointMap(),
           rosCoreURI, simulatedSensorCommunicator,
           ROS2Tools.getControllerOutputTopic(robotModel.getRobotDefinition().getName()).withTypeName(RobotConfigurationData.class), pubSubImplementation);
   }

   public RosModule(FullRobotModelFactory robotModelFactory, RobotROSClockCalculator rosClockCalculator,
                    AvatarRobotRosVisionSensorInformation sensorInformation, HumanoidForceSensorInformation forceSensorInformation, JointNameMap<?> jointMap,
                    URI rosCoreURI, ObjectCommunicator simulatedSensorCommunicator, ROS2Topic<?> robotConfigurationDataTopicName,
                    RealtimeROS2Node realtimeROS2Node)
   {
      this(robotModelFactory, rosClockCalculator, sensorInformation, forceSensorInformation, jointMap, rosCoreURI, simulatedSensorCommunicator,
           robotConfigurationDataTopicName, realtimeROS2Node, null);
   }

   public RosModule(FullRobotModelFactory robotModelFactory, RobotROSClockCalculator rosClockCalculator,
                    AvatarRobotRosVisionSensorInformation sensorInformation, HumanoidForceSensorInformation forceSensorInformation, JointNameMap<?> jointMap,
                    URI rosCoreURI, ObjectCommunicator simulatedSensorCommunicator, ROS2Topic<?> robotConfigurationDataTopicName,
                    PubSubImplementation pubSubImplementation)
   {
      this(robotModelFactory, rosClockCalculator, sensorInformation, forceSensorInformation, jointMap, rosCoreURI, simulatedSensorCommunicator,
           robotConfigurationDataTopicName, null, pubSubImplementation);
   }

   private RosModule(FullRobotModelFactory robotModelFactory, RobotROSClockCalculator rosClockCalculator,
                     AvatarRobotRosVisionSensorInformation sensorInformation, HumanoidForceSensorInformation forceSensorInformation, JointNameMap<?> jointMap,
                     URI rosCoreURI, ObjectCommunicator simulatedSensorCommunicator, ROS2Topic<?> robotConfigurationDataTopicName,
                     RealtimeROS2Node realtimeROS2Node, PubSubImplementation pubSubImplementation)
   {
      LogTools.info("Starting ROS Module");

      String simpleRobotName = robotModelFactory.getRobotDefinition().getName();
      manageROS2Node = realtimeROS2Node == null;
      if (realtimeROS2Node == null)
         realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(pubSubImplementation, "ihmc_ros_node");
      ros2Node = realtimeROS2Node;
      rosMainNode = new RosMainNode(rosCoreURI, ROS_NODE_NAME, true);
      robotName = simpleRobotName.toLowerCase();
      String rosTopicPrefix = "/ihmc_ros/" + robotName;

      this.rosClockCalculator = rosClockCalculator;
      this.rosClockCalculator.subscribeToROS1Topics(rosMainNode);
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    RobotConfigurationData.class,
                                                    robotConfigurationDataTopicName,
                                                    s -> this.rosClockCalculator.receivedRobotConfigurationData(s.takeNextData()));

      this.sensorInformation = sensorInformation;

      RosTfPublisher tfPublisher = new RosTfPublisher(rosMainNode, null);

      robotConfigurationPublisher = new RosRobotConfigurationDataPublisher(robotModelFactory,
                                                                           ros2Node,
                                                                           robotConfigurationDataTopicName,
                                                                           rosMainNode,
                                                                           rosClockCalculator,
                                                                           this.sensorInformation,
                                                                           forceSensorInformation,
                                                                           jointMap,
                                                                           rosTopicPrefix,
                                                                           tfPublisher);

      if (simulatedSensorCommunicator != null)
      {
         publishSimulatedCameraAndLidar(robotModelFactory.createFullRobotModel(), sensorInformation, simulatedSensorCommunicator);

         AvatarRobotLidarParameters[] lidarParameters = sensorInformation.getLidarParameters();
         if (lidarParameters.length > 0)
         {
            AvatarRobotLidarParameters primaryLidar = lidarParameters[0];
            robotConfigurationPublisher.setAdditionalJointStatePublishing(primaryLidar.getLidarSpindleJointTopic(), primaryLidar.getLidarSpindleJointName());
         }
      }

      if (sensorInformation.setupROSLocationService())
      {
         setupRosLocalization();
      }

      //      setupFootstepServiceClient();
      //      setupFootstepPathPlannerService();

      if (CREATE_ROS_ECHO_PUBLISHER)
         setupROSEchoPublisher(rosMainNode, rosTopicPrefix);

      System.out.flush();
      if (manageROS2Node)
         ros2Node.spin();
      rosMainNode.execute();

      LogTools.info("Finished starting ROS Module");
   }

   private void publishSimulatedCameraAndLidar(FullRobotModel fullRobotModel, AvatarRobotVisionSensorInformation sensorInformation,
                                               ObjectCommunicator localObjectCommunicator)
   {
      if (sensorInformation.getCameraParameters().length > 0)
      {
         new RosSCSCameraPublisher(localObjectCommunicator, rosMainNode, rosClockCalculator, sensorInformation.getCameraParameters());
      }

      if (sensorInformation.getLidarParameters().length > 0)
      {
         new RosSCSLidarPublisher(localObjectCommunicator, rosMainNode, rosClockCalculator, fullRobotModel, sensorInformation.getLidarParameters());
      }
   }

   private void setupRosLocalization()
   {
      new IHMCETHRosLocalizationUpdateSubscriber(robotName, rosMainNode, ros2Node, rosClockCalculator::computeRobotMonotonicTime);
      RosLocalizationServiceClient rosLocalizationServiceClient = new RosLocalizationServiceClient(rosMainNode);
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    LocalizationPacket.class,
                                                    ROS2Tools.IHMC_ROOT,
                                                    s -> rosLocalizationServiceClient.receivedPacket(s.takeNextData()));
   }

   //   private void setupFootstepServiceClient()
   //   {
   //      RosFootstepServiceClient rosFootstepServiceClient = new RosFootstepServiceClient(rosModulePacketCommunicator, rosMainNode, physicalProperties.getAnkleHeight());
   //      rosModulePacketCommunicator.attachListener(SnapFootstepPacket.class, rosFootstepServiceClient);
   //   }
   //
   //   private void setupFootstepPathPlannerService()
   //   {
   //      FootstepPlanningParameterization footstepParameters = new BasicFootstepPlanningParameterization();
   //      FootstepPathPlannerService footstepPathPlannerService;
   ////    footstepPathPlannerService = new AStarPathPlannerService(rosMainNode, footstepParameters, physicalProperties.getAnkleHeight(), fieldObjectCommunicator);
   ////    footstepPathPlannerService = new DStarPathPlannerService(rosMainNode, footstepParameters, physicalProperties.getAnkleHeight(), fieldObjectCommunicator);
   //      footstepPathPlannerService = new ADStarPathPlannerService(rosMainNode, footstepParameters, physicalProperties.getAnkleHeight(), rosModulePacketCommunicator);
   //      rosModulePacketCommunicator.attachListener(FootstepPlanRequestPacket.class, footstepPathPlannerService);
   //   }

   @SuppressWarnings({"unchecked", "rawtypes"})
   private void setupROSEchoPublisher(RosMainNode rosMainNode, String namespace)
   {

      PacketCommunicator uiPacketCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.UI_MODULE,
                                                                                                        new IHMCCommunicationKryoNetClassList());
      try
      {
         uiPacketCommunicator.connect();
      }
      catch (IOException e1)
      {
         // local communicator, no error handling neccessary
      }
      NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
      MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();

      Set<Class<?>> inputTypes = GenericROSTranslationTools.getCoreInputTopics();
      for (Class inputType : inputTypes)
      {
         RosMessagePacket rosAnnotation = (RosMessagePacket) inputType.getAnnotation(RosMessagePacket.class);
         String rosMessageClassNameFromIHMCMessage = GenericROSTranslationTools.getRosMessageClassNameFromIHMCMessage(inputType.getSimpleName());
         Message message = messageFactory.newFromType(rosAnnotation.rosPackage() + "/" + rosMessageClassNameFromIHMCMessage);

         IHMCPacketToMsgPublisher<Message, Packet> publisher = IHMCPacketToMsgPublisher.createIHMCPacketToMsgPublisher(message,
                                                                                                                       false,
                                                                                                                       uiPacketCommunicator,
                                                                                                                       inputType);
         String topic = rosAnnotation.topic();
         topic = topic.replaceFirst("control", "output");
         rosMainNode.attachPublisher(namespace + topic, publisher);
      }
   }

   @Override
   public void closeAndDispose()
   {
      robotConfigurationPublisher.closeAndDispose();
      this.rosMainNode.shutdown();
      if (manageROS2Node)
         this.ros2Node.destroy();
   }
}
