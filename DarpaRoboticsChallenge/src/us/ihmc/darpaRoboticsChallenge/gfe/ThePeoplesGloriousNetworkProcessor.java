package us.ihmc.darpaRoboticsChallenge.gfe;

import java.io.IOException;
import java.net.URI;
import java.util.ArrayList;
import java.util.Map;

import org.ros.internal.message.Message;
import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.net.AtomicSettableTimestampProvider;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.ControllerCrashNotificationPacket;
import us.ihmc.communication.packets.InvalidPacketNotificationPacket;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.ros.RosRobotConfigurationDataPublisher;
import us.ihmc.darpaRoboticsChallenge.ros.RosSCSCameraPublisher;
import us.ihmc.darpaRoboticsChallenge.ros.RosSCSLidarPublisher;
import us.ihmc.darpaRoboticsChallenge.ros.RosTfPublisher;
import us.ihmc.ihmcPerception.RosLocalizationPoseCorrectionSubscriber;
import us.ihmc.pathGeneration.footstepGenerator.TimestampedPoseFootStepGenerator;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.ros.PPSTimestampOffsetProvider;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.msgToPacket.IHMCRosApiMessageMap;
import us.ihmc.utilities.ros.publisher.IHMCPacketToMsgPublisher;
import us.ihmc.utilities.ros.publisher.PrintStreamToRosBridge;
import us.ihmc.utilities.ros.publisher.RosTopicPublisher;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;
import us.ihmc.utilities.ros.subscriber.IHMCMsgToPacketSubscriber;

public class ThePeoplesGloriousNetworkProcessor
{
   private static final String nodeName = "/controller";

   private final AtomicSettableTimestampProvider timestampProvider = new AtomicSettableTimestampProvider();
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final RosMainNode rosMainNode;
   private final DRCRobotModel robotModel;
   private final PacketCommunicator controllerCommunicationBridge;
   private final PacketCommunicator scsSensorCommunicationBridge;

   private final ArrayList<AbstractRosTopicSubscriber<?>> subscribers;
   private final ArrayList<RosTopicPublisher<?>> publishers;

   private final NodeConfiguration nodeConfiguration;
   private final MessageFactory messageFactory;
   private final FullRobotModel fullRobotModel;

   public ThePeoplesGloriousNetworkProcessor(URI rosUri, PacketCommunicator controllerCommunicationBridge, PacketCommunicator scsSensorCommunicationBridge, PPSTimestampOffsetProvider ppsOffsetProvider, 
                                             DRCRobotModel robotModel, String namespace) throws IOException
   {
      this.rosMainNode = new RosMainNode(rosUri, namespace + nodeName);
      this.robotModel = robotModel;
      this.controllerCommunicationBridge = controllerCommunicationBridge;
      this.scsSensorCommunicationBridge = scsSensorCommunicationBridge;
      this.ppsTimestampOffsetProvider = ppsOffsetProvider;
      this.ppsTimestampOffsetProvider.attachToRosMainNode(rosMainNode);
      this.subscribers = new ArrayList<AbstractRosTopicSubscriber<?>>();
      this.publishers = new ArrayList<RosTopicPublisher<?>>();

      this.nodeConfiguration = NodeConfiguration.newPrivate();
      this.messageFactory = nodeConfiguration.getTopicMessageFactory();
      this.fullRobotModel = robotModel.createFullRobotModel();
      RobotDataReceiver robotDataReceiver = new RobotDataReceiver(fullRobotModel, null);
      ReferenceFrames referenceFrames = robotDataReceiver.getReferenceFrames();
      controllerCommunicationBridge.attachListener(RobotConfigurationData.class, robotDataReceiver);
      
      setupInputs(namespace, robotDataReceiver, fullRobotModel);
      setupOutputs(namespace);
      setupRosLocalization();
      setupErrorTopics();

      rosMainNode.execute();

      while(!rosMainNode.isStarted())
      {
         ThreadTools.sleep(100);
      }

      controllerCommunicationBridge.connect();
      
      System.out.println("IHMC ROS API node successfully connected to controller.");
   }

   public ThePeoplesGloriousNetworkProcessor(URI rosUri, PacketCommunicator controllerCommunicationBridge, DRCRobotModel robotModel, String namespace) throws
         IOException
   {
      this(rosUri, controllerCommunicationBridge, null, robotModel.getPPSTimestampOffsetProvider(), robotModel, namespace);
   }

   private void setupOutputs(String namespace)
   {
      SDFFullRobotModel fullRobotModel = robotModel.createFullRobotModel();
      DRCRobotSensorInformation sensorInformation = robotModel.getSensorInformation();

      RosTfPublisher tfPublisher = new RosTfPublisher(rosMainNode);

      new RosRobotConfigurationDataPublisher(robotModel, controllerCommunicationBridge, rosMainNode, ppsTimestampOffsetProvider, namespace, tfPublisher);

      if(scsSensorCommunicationBridge != null)
      {
         publishSimulatedCameraAndLidar(fullRobotModel, sensorInformation, tfPublisher);
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

   private void publishSimulatedCameraAndLidar(SDFFullRobotModel fullRobotModel, DRCRobotSensorInformation sensorInformation, RosTfPublisher tfPublisher)
   {
      if (sensorInformation.getCameraParameters().length > 0)
      {
         new RosSCSCameraPublisher(scsSensorCommunicationBridge, rosMainNode, ppsTimestampOffsetProvider, sensorInformation.getCameraParameters());
      }

      if (sensorInformation.getLidarParameters().length > 0)
      {
         new RosSCSLidarPublisher(scsSensorCommunicationBridge, rosMainNode, ppsTimestampOffsetProvider, fullRobotModel,
               sensorInformation.getLidarParameters(), tfPublisher);
      }
   }

   private void setupInputs(String namespace, RobotDataReceiver robotDataReceiver, FullRobotModel fullRobotModel)
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
   }

   private void setupRosLocalization()
   {
      new RosLocalizationPoseCorrectionSubscriber(rosMainNode, controllerCommunicationBridge, ppsTimestampOffsetProvider);
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
