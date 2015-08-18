package us.ihmc.darpaRoboticsChallenge.gfe;

import org.ros.internal.message.Message;
import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;

import us.ihmc.SdfLoader.SDFBaseFullRobotModel;
import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.*;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.ros.*;
import us.ihmc.ihmcPerception.IHMCProntoRosLocalizationUpdateSubscriber;
import us.ihmc.ihmcPerception.RosLocalizationPoseCorrectionSubscriber;
import us.ihmc.pathGeneration.footstepGenerator.TimestampedPoseFootStepGenerator;
import us.ihmc.sensorProcessing.parameters.DRCRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.humanoidRobotics.model.FullHumanoidRobotModel;
import us.ihmc.utilities.ros.PPSTimestampOffsetProvider;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.msgToPacket.IHMCRosApiMessageMap;
import us.ihmc.utilities.ros.publisher.IHMCPacketToMsgPublisher;
import us.ihmc.utilities.ros.publisher.PrintStreamToRosBridge;
import us.ihmc.utilities.ros.publisher.RosTopicPublisher;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;
import us.ihmc.utilities.ros.subscriber.IHMCMsgToPacketSubscriber;
import us.ihmc.utilities.ros.subscriber.RequestControllerStopSubscriber;
import us.ihmc.utilities.ros.subscriber.RosArmJointTrajectorySubscriber;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

import java.io.IOException;
import java.net.URI;
import java.util.ArrayList;
import java.util.Map;

public class ThePeoplesGloriousNetworkProcessor
{
   private static final String nodeName = "/controller";

   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final RosMainNode rosMainNode;
   private final DRCRobotModel robotModel;
   private final PacketCommunicator controllerCommunicationBridge;
   private final ObjectCommunicator scsSensorCommunicationBridge;

   private final ArrayList<AbstractRosTopicSubscriber<?>> subscribers;
   private final ArrayList<RosTopicPublisher<?>> publishers;

   private final NodeConfiguration nodeConfiguration;
   private final MessageFactory messageFactory;
   private final FullHumanoidRobotModel fullRobotModel;

   public ThePeoplesGloriousNetworkProcessor(URI rosUri, PacketCommunicator gfe_communicator, ObjectCommunicator sensorCommunicator, PPSTimestampOffsetProvider ppsOffsetProvider, 
                                             DRCRobotModel robotModel, String namespace, String tfPrefix) throws IOException
   {
      this.rosMainNode = new RosMainNode(rosUri, namespace + nodeName);
      this.robotModel = robotModel;
      this.controllerCommunicationBridge = gfe_communicator;
      this.scsSensorCommunicationBridge = sensorCommunicator;
      this.ppsTimestampOffsetProvider = ppsOffsetProvider;
      this.ppsTimestampOffsetProvider.attachToRosMainNode(rosMainNode);
      this.subscribers = new ArrayList<AbstractRosTopicSubscriber<?>>();
      this.publishers = new ArrayList<RosTopicPublisher<?>>();

      this.nodeConfiguration = NodeConfiguration.newPrivate();
      this.messageFactory = nodeConfiguration.getTopicMessageFactory();
      this.fullRobotModel = robotModel.createFullRobotModel();
      RobotDataReceiver robotDataReceiver = new RobotDataReceiver(fullRobotModel, null);
      gfe_communicator.attachListener(RobotConfigurationData.class, robotDataReceiver);
      gfe_communicator.attachListener(RobotConfigurationData.class, ppsOffsetProvider);
      gfe_communicator.attachListener(HighLevelStateChangePacket.class, new PeriodicRosHighLevelStatePublisher(rosMainNode, namespace));
      gfe_communicator.attachListener(CapturabilityBasedStatus.class, new RosCapturabilityBasedStatusPublisher(rosMainNode, namespace));

      setupInputs(namespace, robotDataReceiver, fullRobotModel);
      setupOutputs(namespace, tfPrefix);
      setupRosLocalization();
      setupErrorTopics();

      rosMainNode.execute();

      while(!rosMainNode.isStarted())
      {
         ThreadTools.sleep(100);
      }

      gfe_communicator.connect();
      
      System.out.println("IHMC ROS API node successfully started.");
   }

   public ThePeoplesGloriousNetworkProcessor(URI rosUri, PacketCommunicator controllerCommunicationBridge, DRCRobotModel robotModel, String namespace, String tfPrefix) throws
         IOException
   {
      this(rosUri, controllerCommunicationBridge, null, robotModel.getPPSTimestampOffsetProvider(), robotModel, namespace, tfPrefix);
   }

   private void setupOutputs(String namespace, String tfPrefix)
   {
      SDFBaseFullRobotModel fullRobotModel = robotModel.createFullRobotModel();
      DRCRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
      DRCRobotJointMap jointMap = robotModel.getJointMap();

      RosTfPublisher tfPublisher = new RosTfPublisher(rosMainNode, tfPrefix);

      RosRobotConfigurationDataPublisher robotConfigurationPublisher = new RosRobotConfigurationDataPublisher(robotModel, controllerCommunicationBridge,
            rosMainNode, ppsTimestampOffsetProvider, sensorInformation, jointMap, namespace, tfPublisher);
      if(scsSensorCommunicationBridge != null)
      {
         publishSimulatedCameraAndLidar(fullRobotModel, sensorInformation, robotConfigurationPublisher);
      }

      Map<String, Class> outputPacketList = IHMCRosApiMessageMap.OUTPUT_PACKET_MESSAGE_NAME_MAP;

      for (Map.Entry<String, Class> e : outputPacketList.entrySet())
      {
         Message message = messageFactory.newFromType(e.getKey());

         IHMCPacketToMsgPublisher<Message, Packet> publisher = IHMCPacketToMsgPublisher.createIHMCPacketToMsgPublisher(message, false,
               controllerCommunicationBridge, e.getValue());
         publishers.add(publisher);
         rosMainNode.attachPublisher(namespace + IHMCRosApiMessageMap.PACKET_TO_TOPIC_MAP.get(e.getValue()), publisher);
      }

      PrintStreamToRosBridge printStreamBridge = new PrintStreamToRosBridge(rosMainNode, namespace);
      printStreamBridge.start();
      System.setErr(printStreamBridge);
   }

   private void publishSimulatedCameraAndLidar(SDFBaseFullRobotModel fullRobotModel, DRCRobotSensorInformation sensorInformation, RosRobotConfigurationDataPublisher robotConfigurationPublisher)
   {
      if (sensorInformation.getCameraParameters().length > 0)
      {
         new RosSCSCameraPublisher(scsSensorCommunicationBridge, rosMainNode, ppsTimestampOffsetProvider, sensorInformation.getCameraParameters());
      }

      DRCRobotLidarParameters[] lidarParameters = sensorInformation.getLidarParameters();
      if (lidarParameters.length > 0)
      {
         new RosSCSLidarPublisher(scsSensorCommunicationBridge, rosMainNode, ppsTimestampOffsetProvider, fullRobotModel, lidarParameters);
         
         DRCRobotLidarParameters primaryLidar = lidarParameters[0];
         robotConfigurationPublisher.setAdditionalJointStatePublishing(primaryLidar.getLidarSpindleJointTopic(), primaryLidar.getLidarSpindleJointName());
      }
   }

   private void setupInputs(String namespace, RobotDataReceiver robotDataReceiver, FullHumanoidRobotModel fullRobotModel)
   {
      Map<String, Class> inputPacketList = IHMCRosApiMessageMap.INPUT_PACKET_MESSAGE_NAME_MAP;

      for (Map.Entry<String, Class> e : inputPacketList.entrySet())
      {
         Message message = messageFactory.newFromType(e.getKey());
         IHMCMsgToPacketSubscriber<Message> subscriber = IHMCMsgToPacketSubscriber.createIHMCMsgToPacketSubscriber(message,
               controllerCommunicationBridge, PacketDestination.CONTROLLER.ordinal());
         subscribers.add(subscriber);
         rosMainNode.attachSubscriber(namespace + IHMCRosApiMessageMap.PACKET_TO_TOPIC_MAP.get(e.getValue()), subscriber);
      }
      
      TimestampedPoseFootStepGenerator footPoseGenerator = new TimestampedPoseFootStepGenerator(robotDataReceiver, fullRobotModel, controllerCommunicationBridge);
      rosMainNode.attachSubscriber(namespace + "/control/endpoint_footstep_generator", footPoseGenerator);
      
      RosArmJointTrajectorySubscriber rosJointTrajectorySubscriber = new RosArmJointTrajectorySubscriber(controllerCommunicationBridge, fullRobotModel);
      rosMainNode.attachSubscriber(namespace + "/control/arm_joint_trajectory2", rosJointTrajectorySubscriber);
      
      RequestControllerStopSubscriber requestStopSubscriber = new RequestControllerStopSubscriber(controllerCommunicationBridge);
      rosMainNode.attachSubscriber(namespace + "/control/request_stop", requestStopSubscriber);
   }

   private void setupRosLocalization()
   {
      new RosLocalizationPoseCorrectionSubscriber(rosMainNode, controllerCommunicationBridge, ppsTimestampOffsetProvider);
      new IHMCProntoRosLocalizationUpdateSubscriber(rosMainNode, controllerCommunicationBridge, ppsTimestampOffsetProvider);
   }
   
   private void setupErrorTopics()
   {
      controllerCommunicationBridge.attachListener(InvalidPacketNotificationPacket.class, new PacketConsumer<InvalidPacketNotificationPacket>()
      {
         @Override
         public void receivedPacket(InvalidPacketNotificationPacket packet)
         {
            System.err.println("Controller recieved invalid packet of type " + packet.packetClass);
            System.err.println("Message: " + packet.errorMessage);
         }
      });
      
      controllerCommunicationBridge.attachListener(ControllerCrashNotificationPacket.class, new PacketConsumer<ControllerCrashNotificationPacket>()
      {
         @Override
         public void receivedPacket(ControllerCrashNotificationPacket packet)
         {
            System.err.println("Controller crashed at " + packet.location);
            System.err.println("StackTrace: " + packet.stacktrace);
         }
      });
   }
}
