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
import us.ihmc.avatar.ros.DRCROSPPSTimestampOffsetProvider;
import us.ihmc.avatar.ros.IHMCPacketToMsgPublisher;
import us.ihmc.avatar.ros.RosRobotConfigurationDataPublisher;
import us.ihmc.avatar.ros.RosSCSCameraPublisher;
import us.ihmc.avatar.ros.RosSCSLidarPublisher;
import us.ihmc.avatar.ros.RosTfPublisher;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.ihmcPerception.IHMCETHRosLocalizationUpdateSubscriber;
import us.ihmc.ihmcPerception.RosLocalizationServiceClient;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.FullRobotModelFactory;
import us.ihmc.robotics.partNames.JointNameMap;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.sensorProcessing.parameters.AvatarRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.AvatarRobotRosVisionSensorInformation;
import us.ihmc.sensorProcessing.parameters.AvatarRobotVisionSensorInformation;
import us.ihmc.sensorProcessing.parameters.HumanoidForceSensorInformation;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.msgToPacket.converter.GenericROSTranslationTools;

public class RosModule
{
   private static final boolean DEBUG = false;
   private static final boolean CREATE_ROS_ECHO_PUBLISHER = false;

   private static final String ROS_NODE_NAME = "networkProcessor/rosModule";

   private final Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "ihmc_ros_node");

   private final RosMainNode rosMainNode;
   private final DRCROSPPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final AvatarRobotRosVisionSensorInformation sensorInformation;
   private final String robotName;

   public RosModule(DRCRobotModel robotModel, URI rosCoreURI, ObjectCommunicator simulatedSensorCommunicator)
   {
      this(robotModel, robotModel.getPPSTimestampOffsetProvider(), robotModel.getSensorInformation(), robotModel.getSensorInformation(),
           robotModel.getJointMap(), rosCoreURI, simulatedSensorCommunicator,
           ControllerAPIDefinition.getPublisherTopicNameGenerator(robotModel.getRobotDescription().getName()).generateTopicName(RobotConfigurationData.class));
   }

   public RosModule(FullRobotModelFactory robotModelFactory, DRCROSPPSTimestampOffsetProvider ppsTimestampOffsetProvider,
                    AvatarRobotRosVisionSensorInformation sensorInformation, HumanoidForceSensorInformation forceSensorInformation, JointNameMap jointMap,
                    URI rosCoreURI, ObjectCommunicator simulatedSensorCommunicator, String robotConfigurationDataTopicName)
   {
      String simpleRobotName = robotModelFactory.getRobotDescription().getName();
      rosMainNode = new RosMainNode(rosCoreURI, ROS_NODE_NAME, true);
      robotName = simpleRobotName.toLowerCase();
      String rosTopicPrefix = "/ihmc_ros/" + robotName;

      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.ppsTimestampOffsetProvider.attachToRosMainNode(rosMainNode);
      ROS2Tools.createCallbackSubscription(ros2Node, RobotConfigurationData.class, robotConfigurationDataTopicName,
                                           s -> this.ppsTimestampOffsetProvider.receivedPacket(s.takeNextData()));

      this.sensorInformation = sensorInformation;

      RosTfPublisher tfPublisher = new RosTfPublisher(rosMainNode, null);

      RosRobotConfigurationDataPublisher robotConfigurationPublisher = new RosRobotConfigurationDataPublisher(robotModelFactory, ros2Node, robotConfigurationDataTopicName,
                                                                                                              rosMainNode, ppsTimestampOffsetProvider,
                                                                                                              this.sensorInformation, forceSensorInformation,
                                                                                                              jointMap, rosTopicPrefix, tfPublisher);

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
      rosMainNode.execute();
      if (DEBUG)
         PrintTools.debug("Finished creating ROS Module.");
   }

   private void publishSimulatedCameraAndLidar(FullRobotModel fullRobotModel, AvatarRobotVisionSensorInformation sensorInformation,
                                               ObjectCommunicator localObjectCommunicator)
   {
      if (sensorInformation.getCameraParameters().length > 0)
      {
         new RosSCSCameraPublisher(localObjectCommunicator, rosMainNode, ppsTimestampOffsetProvider, sensorInformation.getCameraParameters());
      }

      if (sensorInformation.getLidarParameters().length > 0)
      {
         new RosSCSLidarPublisher(localObjectCommunicator, rosMainNode, ppsTimestampOffsetProvider, fullRobotModel, sensorInformation.getLidarParameters());
      }
   }

   private void setupRosLocalization()
   {
      new IHMCETHRosLocalizationUpdateSubscriber(robotName, rosMainNode, ros2Node, ppsTimestampOffsetProvider);
      RosLocalizationServiceClient rosLocalizationServiceClient = new RosLocalizationServiceClient(rosMainNode);
      ROS2Tools.createCallbackSubscription(ros2Node, LocalizationPacket.class, ROS2Tools.getDefaultTopicNameGenerator(),
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

   @SuppressWarnings("unchecked")
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

         IHMCPacketToMsgPublisher<Message, Packet> publisher = IHMCPacketToMsgPublisher.createIHMCPacketToMsgPublisher(message, false, uiPacketCommunicator,
                                                                                                                       inputType);
         String topic = rosAnnotation.topic();
         topic = topic.replaceFirst("control", "output");
         rosMainNode.attachPublisher(namespace + topic, publisher);
      }
   }
}
