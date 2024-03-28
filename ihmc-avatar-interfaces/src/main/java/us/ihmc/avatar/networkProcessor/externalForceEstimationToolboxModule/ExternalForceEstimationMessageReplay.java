package us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import controller_msgs.msg.dds.*;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.RobotConfigurationDataPubSubType;
import toolbox_msgs.msg.dds.ExternalForceEstimationConfigurationMessage;
import toolbox_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;

import javax.swing.*;
import javax.swing.filechooser.FileNameExtensionFilter;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;

public class ExternalForceEstimationMessageReplay
{
   static final String timestampName = "Timestamp";
   static final String robotConfigurationDataName = RobotConfigurationData.class.getSimpleName();
   static final String robotDesiredConfigurationDataName = RobotDesiredConfigurationData.class.getSimpleName();

   private final String robotName;
   private final List<MessageSet> messages;
   private final RealtimeROS2Node ros2Node;

   private final JSONSerializer<RobotConfigurationData> robotConfigurationDataSerializer = new JSONSerializer<>(new RobotConfigurationDataPubSubType());
   private final JSONSerializer<RobotDesiredConfigurationData> robotDesiredConfigurationDataSerializer = new JSONSerializer<>(new RobotDesiredConfigurationDataPubSubType());

   private final ROS2PublisherBasics<RobotConfigurationData> robotConfigurationDataPublisher;
   private final ROS2PublisherBasics<RobotDesiredConfigurationData> robotDesiredConfigurationDataPublisher;
   private final ROS2PublisherBasics<ToolboxStateMessage> toolboxStatePublisher;
   private final ROS2PublisherBasics<ExternalForceEstimationConfigurationMessage> configMessagePublisher;

   public ExternalForceEstimationMessageReplay(String robotName, InputStream inputStream, PubSubImplementation pubSubImplementation) throws IOException
   {
      this.robotName = robotName;
      messages = loadMessages(inputStream);

      String name = getClass().getSimpleName();
      ros2Node = ROS2Tools.createRealtimeROS2Node(pubSubImplementation, "ihmc_" + name);

      ROS2Topic controllerOutputTopic = HumanoidControllerAPI.getOutputTopic(robotName);
      robotConfigurationDataPublisher = ros2Node.createPublisher(ROS2Tools.typeNamedTopic(RobotConfigurationData.class).withTopic(controllerOutputTopic));
      robotDesiredConfigurationDataPublisher = ros2Node.createPublisher(ROS2Tools.typeNamedTopic(RobotDesiredConfigurationData.class)
                                                                                 .withTopic(controllerOutputTopic));

      ROS2Topic toolboxInputTopic = ExternalForceEstimationToolboxModule.getInputTopic(robotName);
      configMessagePublisher = ros2Node.createPublisher(ROS2Tools.typeNamedTopic(ExternalForceEstimationConfigurationMessage.class)
                                                                 .withTopic(toolboxInputTopic));
      toolboxStatePublisher = ros2Node.createPublisher(ROS2Tools.typeNamedTopic(ToolboxStateMessage.class).withTopic(toolboxInputTopic));

      ros2Node.spin();
   }

   public void replayAllMessages()
   {
      ExternalForceEstimationConfigurationMessage configurationMessage = new ExternalForceEstimationConfigurationMessage();
      configurationMessage.setEstimatorGain(0.75);

      // Valkyrie pelvis
      //      configurationMessage.setEndEffectorHashCode(-878626891);
      //      configurationMessage.getContactPointPositions().setToZero();

      // Valkyrie right elbow pitch
      configurationMessage.getRigidBodyHashCodes().add(601127246);
      configurationMessage.getContactPointPositions().add().set(0.0, -0.35, -0.03);

      configurationMessage.setCalculateRootJointWrench(true);
      configMessagePublisher.publish(configurationMessage);

      sendToolboxStateMessage(ToolboxState.WAKE_UP);
      ThreadTools.sleep(1);

      for (int i = 0; i < messages.size(); i++)
      {
         sendMessagesAtIndex(i);

         if(i != messages.size() - 1)
         {
            int timeDifferenceMillis = (int) (1e-6 * (messages.get(i + 1).timestamp - messages.get(i).timestamp));
            ThreadTools.sleep(timeDifferenceMillis);
         }
      }

      // send sleep packet
      sendToolboxStateMessage(ToolboxState.SLEEP);
   }

   private List<MessageSet> loadMessages(InputStream inputStream) throws IOException
   {
      ObjectMapper objectMapper = new ObjectMapper();
      JsonNode jsonNode = objectMapper.readTree(inputStream);
      int size = jsonNode.size();

      List<MessageSet> allMessages = new ArrayList<>();

      for (int i = 0; i < size; i++)
      {
         JsonNode childNode = jsonNode.get(i);
         MessageSet messageSet = new MessageSet(childNode.get(timestampName).asLong());

         if (childNode.has(robotConfigurationDataName))
         {
            messageSet.robotConfigurationData = robotConfigurationDataSerializer.deserialize(childNode.get(robotConfigurationDataName).toString());
         }
         if (childNode.has(robotDesiredConfigurationDataName))
         {
            messageSet.robotDesiredConfigurationData = robotDesiredConfigurationDataSerializer.deserialize(childNode.get(robotDesiredConfigurationDataName)
                                                                                                                    .toString());
         }

         allMessages.add(messageSet);
      }

      return allMessages;
   }

   private void sendMessagesAtIndex(int i)
   {
      MessageSet messageSet = messages.get(i);

      if (messageSet.robotConfigurationData != null)
      {
         robotConfigurationDataPublisher.publish(messageSet.robotConfigurationData);
      }
      if (messageSet.robotDesiredConfigurationData != null)
      {
         robotDesiredConfigurationDataPublisher.publish(messageSet.robotDesiredConfigurationData);
      }
   }

   private void sendToolboxStateMessage(ToolboxState toolboxState)
   {
      ToolboxStateMessage toolboxStateMessage = new ToolboxStateMessage();
      toolboxStateMessage.setRequestedToolboxState(toolboxState.toByte());
      toolboxStatePublisher.publish(toolboxStateMessage);
   }

   private class MessageSet
   {
      private final long timestamp;
      private RobotConfigurationData robotConfigurationData = null;
      private RobotDesiredConfigurationData robotDesiredConfigurationData = null;

      MessageSet(long timestamp)
      {
         this.timestamp = timestamp;
      }
   }

   public static void main(String[] args) throws IOException
   {
      String robotName = "Valkyrie"; // "Atlas"; //

      JFileChooser fileChooser = new JFileChooser();
      fileChooser.setFileFilter(new FileNameExtensionFilter("JSON log", "json"));
      int chooserState = fileChooser.showOpenDialog(null);

      if (chooserState == JFileChooser.APPROVE_OPTION)
      {
         InputStream inputStream = new FileInputStream(fileChooser.getSelectedFile());
         new ExternalForceEstimationMessageReplay(robotName, inputStream, PubSubImplementation.FAST_RTPS).replayAllMessages();
      }
   }
}
