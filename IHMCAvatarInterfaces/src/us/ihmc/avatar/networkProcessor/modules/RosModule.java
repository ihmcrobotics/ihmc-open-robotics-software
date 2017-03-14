package us.ihmc.avatar.networkProcessor.modules;

import java.io.IOException;
import java.net.URI;
import java.util.Set;

import org.ros.internal.message.Message;
import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.ros.DRCROSPPSTimestampOffsetProvider;
import us.ihmc.avatar.ros.IHMCPacketToMsgPublisher;
import us.ihmc.avatar.ros.RosRobotConfigurationDataPublisher;
import us.ihmc.avatar.ros.RosSCSCameraPublisher;
import us.ihmc.avatar.ros.RosSCSLidarPublisher;
import us.ihmc.avatar.ros.RosTfPublisher;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.communication.packets.sensing.LocalizationPacket;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.ihmcPerception.IHMCETHRosLocalizationUpdateSubscriber;
import us.ihmc.ihmcPerception.RosLocalizationServiceClient;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.sensorProcessing.parameters.DRCRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.msgToPacket.converter.GenericROSTranslationTools;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class RosModule
{
   private static final boolean DEBUG = false;
   private static final boolean CREATE_ROS_ECHO_PUBLISHER = false;

   private static final String ROS_NODE_NAME = "networkProcessor/rosModule";

//   private final KryoLocalPacketCommunicator rosModulePacketCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(),
//         PacketDestination.ROS_MODULE.ordinal(), "RosModule");

   private final PacketCommunicator rosModulePacketCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.ROS_MODULE,
         new IHMCCommunicationKryoNetClassList());

   private final RosMainNode rosMainNode;
   private final DRCROSPPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final DRCRobotSensorInformation sensorInformation;

   public RosModule(DRCRobotModel robotModel, URI rosCoreURI, ObjectCommunicator simulatedSensorCommunicator)
   {
      rosMainNode = new RosMainNode(rosCoreURI, ROS_NODE_NAME, true);
      String rosTopicPrefix = "/ihmc_ros/" + robotModel.getSimpleRobotName().toLowerCase();

      ppsTimestampOffsetProvider = robotModel.getPPSTimestampOffsetProvider();
      ppsTimestampOffsetProvider.attachToRosMainNode(rosMainNode);
      rosModulePacketCommunicator.attachListener(RobotConfigurationData.class, ppsTimestampOffsetProvider);

      sensorInformation = robotModel.getSensorInformation();

      RosTfPublisher tfPublisher = new RosTfPublisher(rosMainNode, null);

      DRCRobotJointMap jointMap = robotModel.getJointMap();
      RosRobotConfigurationDataPublisher robotConfigurationPublisher = new RosRobotConfigurationDataPublisher(robotModel, rosModulePacketCommunicator,
            rosMainNode, ppsTimestampOffsetProvider, sensorInformation, jointMap, rosTopicPrefix, tfPublisher);

      if(simulatedSensorCommunicator != null)
      {
         publishSimulatedCameraAndLidar(robotModel.createFullRobotModel(), sensorInformation, simulatedSensorCommunicator);

         DRCRobotLidarParameters[] lidarParameters = sensorInformation.getLidarParameters();
         if (lidarParameters.length > 0)
         {
            DRCRobotLidarParameters primaryLidar = lidarParameters[0];
            robotConfigurationPublisher.setAdditionalJointStatePublishing(primaryLidar.getLidarSpindleJointTopic(), primaryLidar.getLidarSpindleJointName());
         }
      }

      if(sensorInformation.setupROSLocationService())
      {
         setupRosLocalization();
      }

//      setupFootstepServiceClient();
//      setupFootstepPathPlannerService();

      if (CREATE_ROS_ECHO_PUBLISHER)
         setupROSEchoPublisher(rosMainNode, rosTopicPrefix);

      try
      {
         rosModulePacketCommunicator.connect();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

      System.out.flush();
      rosMainNode.execute();
      if (DEBUG)
         PrintTools.debug("Finished creating ROS Module.");
   }

   private void publishSimulatedCameraAndLidar(FullRobotModel fullRobotModel, DRCRobotSensorInformation sensorInformation, ObjectCommunicator localObjectCommunicator)
   {
      if (sensorInformation.getCameraParameters().length > 0)
      {
         new RosSCSCameraPublisher(localObjectCommunicator, rosMainNode, ppsTimestampOffsetProvider, sensorInformation.getCameraParameters());
      }

      if (sensorInformation.getLidarParameters().length > 0)
      {
         new RosSCSLidarPublisher(localObjectCommunicator, rosMainNode, ppsTimestampOffsetProvider, fullRobotModel,
               sensorInformation.getLidarParameters());
      }
   }

   private void setupRosLocalization()
   {
      new IHMCETHRosLocalizationUpdateSubscriber(rosMainNode, rosModulePacketCommunicator, ppsTimestampOffsetProvider);
      RosLocalizationServiceClient rosLocalizationServiceClient = new RosLocalizationServiceClient(rosMainNode);
      rosModulePacketCommunicator.attachListener(LocalizationPacket.class, rosLocalizationServiceClient);
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

      PacketCommunicator uiPacketCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.UI_MODULE, new IHMCCommunicationKryoNetClassList());
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

         IHMCPacketToMsgPublisher<Message, Packet> publisher = IHMCPacketToMsgPublisher.createIHMCPacketToMsgPublisher(message, false,
               uiPacketCommunicator, inputType);
         String topic = rosAnnotation.topic();
         topic = topic.replaceFirst("control", "output");
         rosMainNode.attachPublisher(namespace + topic, publisher);
      }
   }
}
