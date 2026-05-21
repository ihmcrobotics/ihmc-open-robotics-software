package us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule;

import us.ihmc.communication.ROS2Tools;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import controller_msgs.RobotConfigurationData;
import controller_msgs.RobotDesiredConfigurationData;
import toolbox_msgs.ExternalForceEstimationConfigurationMessage;
import toolbox_msgs.ToolboxStateMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.communication.serialization.ROS2MessageCdrFileTools;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.jros2.AsyncROS2Node;

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
   private final AsyncROS2Node ros2Node;

   private final ROS2Publisher<RobotConfigurationData> robotConfigurationDataPublisher;
   private final ROS2Publisher<RobotDesiredConfigurationData> robotDesiredConfigurationDataPublisher;
   private final ROS2Publisher<ToolboxStateMessage> toolboxStatePublisher;
   private final ROS2Publisher<ExternalForceEstimationConfigurationMessage> configMessagePublisher;

   public ExternalForceEstimationMessageReplay(String robotName, InputStream inputStream) throws IOException
   {
      this.robotName = robotName;
      messages = loadMessages(inputStream);

      String name = getClass().getSimpleName();
      ros2Node = new AsyncROS2Node("ihmc_" + name);

      ROS2Topic controllerOutputTopic = HumanoidControllerAPI.getOutputTopic(robotName);
      robotConfigurationDataPublisher = ros2Node.createPublisher(controllerOutputTopic.withType(RobotConfigurationData.class));
      robotDesiredConfigurationDataPublisher = ros2Node.createPublisher(controllerOutputTopic.withType(RobotDesiredConfigurationData.class));

      ROS2Topic toolboxInputTopic = ExternalForceEstimationToolboxModule.getInputTopic(robotName);
      configMessagePublisher = ros2Node.createPublisher(toolboxInputTopic.withType(ExternalForceEstimationConfigurationMessage.class));
      toolboxStatePublisher = ros2Node.createPublisher(toolboxInputTopic.withType(ToolboxStateMessage.class));

      ROS2Tools.blockUntilInterrupted();
   }

   public void replayAllMessages()
   {
      ExternalForceEstimationConfigurationMessage configurationMessage = new ExternalForceEstimationConfigurationMessage();
      configurationMessage.setEstimatorGain(0.75);

      configurationMessage.getRigidBodyHashCodes().add(601127246);
      configurationMessage.getContactPointPositions().add().getPoint().set(0.0, -0.35, -0.03);

      configurationMessage.setCalculateRootJointWrench(true);
      configMessagePublisher.publish(configurationMessage);

      sendToolboxStateMessage(ToolboxState.WAKE_UP);
      ThreadTools.sleep(1);

      for (int i = 0; i < messages.size(); i++)
      {
         sendMessagesAtIndex(i);

         if (i != messages.size() - 1)
         {
            int timeDifferenceMillis = (int) (1e-6 * (messages.get(i + 1).timestamp - messages.get(i).timestamp));
            ThreadTools.sleep(timeDifferenceMillis);
         }
      }

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
            messageSet.robotConfigurationData = new RobotConfigurationData();
            ROS2MessageCdrFileTools.deserializeFromJsonNode(childNode.get(robotConfigurationDataName), messageSet.robotConfigurationData);
         }
         if (childNode.has(robotDesiredConfigurationDataName))
         {
            messageSet.robotDesiredConfigurationData = new RobotDesiredConfigurationData();
            ROS2MessageCdrFileTools.deserializeFromJsonNode(childNode.get(robotDesiredConfigurationDataName), messageSet.robotDesiredConfigurationData);
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
         new ExternalForceEstimationMessageReplay(robotName, inputStream).replayAllMessages();
      }
   }
}
