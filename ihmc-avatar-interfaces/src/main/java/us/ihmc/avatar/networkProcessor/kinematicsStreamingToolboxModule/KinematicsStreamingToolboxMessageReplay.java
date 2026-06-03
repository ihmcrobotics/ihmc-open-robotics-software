package us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import controller_msgs.CapturabilityBasedStatus;
import controller_msgs.RobotConfigurationData;
import org.apache.commons.lang3.mutable.MutableInt;
import toolbox_msgs.KinematicsStreamingToolboxInputMessage;
import toolbox_msgs.KinematicsToolboxConfigurationMessage;
import toolbox_msgs.ToolboxStateMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.communication.serialization.ROS2MessageCdrFileTools;
import us.ihmc.jros2.ROS2Message;
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

import static us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxMessageLogger.*;

/**
 * Loads and replays messages from file. There are two ways to do replay:
 * 1. Calling {@link #replayAllMessages} will immediately stream all messages, using wall time to determine when to send
 * 2. To use as this as a callback based on an external clock, first call {@link #initialize}, then successive calls to {@link #update}
 */
public class KinematicsStreamingToolboxMessageReplay
{
   private final List<KinematicsStreamingToolboxMessageSet> messages;

   private final ROS2Publisher<RobotConfigurationData> robotConfigurationDataPublisher;
   private final ROS2Publisher<CapturabilityBasedStatus> capturabilityBasedStatusPublisher;
   private final ROS2Publisher<KinematicsToolboxConfigurationMessage> kinematicsToolboxConfigurationPublisher;
   private final ROS2Publisher<KinematicsStreamingToolboxInputMessage> kinematicsStreamingToolboxInputPublisher;
   private final ROS2Publisher<ToolboxStateMessage> toolboxStatePublisher;

   private final MutableInt counter = new MutableInt();
   private double timeOffsetSeconds;

   private final AsyncROS2Node ros2Node;

   public KinematicsStreamingToolboxMessageReplay(String robotName, InputStream inputStream) throws IOException
   {
      messages = loadMessages(inputStream);
      String name = getClass().getSimpleName();

      ros2Node = new AsyncROS2Node("ihmc_" + name);

      ROS2Topic controllerOutputTopic = HumanoidControllerAPI.getOutputTopic(robotName);
      robotConfigurationDataPublisher = ros2Node.createPublisher(controllerOutputTopic.withType(RobotConfigurationData.class));
      capturabilityBasedStatusPublisher = ros2Node.createPublisher(controllerOutputTopic.withType(CapturabilityBasedStatus.class));

      ROS2Topic toolboxInputTopic = KinematicsStreamingToolboxModule.getInputTopic(robotName);
      kinematicsToolboxConfigurationPublisher = ros2Node.createPublisher(toolboxInputTopic.withType(KinematicsToolboxConfigurationMessage.class));
      kinematicsStreamingToolboxInputPublisher = ros2Node.createPublisher(toolboxInputTopic.withType(KinematicsStreamingToolboxInputMessage.class));
      toolboxStatePublisher = ros2Node.createPublisher(toolboxInputTopic.withType(ToolboxStateMessage.class));
   }

   public void replayAllMessages()
   {
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

   /**
    * Initializes replay as a callback given an external time source.
    */
   public void initialize(double timeSeconds)
   {
      counter.setValue(0);
      timeOffsetSeconds = timeSeconds - Conversions.nanosecondsToSeconds(messages.get(0).timestamp);
   }

   /**
    * Broadcasts a message according to the given time. Must be preceeded by a call to initialize
    * Returns whether there is a subsequent message available
    */
   public boolean update(double timeSeconds)
   {
      if(counter.getValue() >= messages.size())
         return false;

      double nextMessageTime = Conversions.nanosecondsToSeconds(messages.get(counter.getValue()).timestamp) + timeOffsetSeconds;
      if(nextMessageTime < timeSeconds)
      {
         sendMessagesAtIndex(counter.getValue());
         counter.increment();
      }

      return counter.getValue() < messages.size();
   }

   public void close()
   {
      ros2Node.close();
   }

   private void sendMessagesAtIndex(int i)
   {
      KinematicsStreamingToolboxMessageSet messageSet = messages.get(i);

      if(messageSet.kinematicsToolboxConfigurationMessage != null)
      {
         kinematicsToolboxConfigurationPublisher.publish(messageSet.kinematicsToolboxConfigurationMessage);
      }
      if(messageSet.kinematicsStreamingToolboxInputMessage != null)
      {
         kinematicsStreamingToolboxInputPublisher.publish(messageSet.kinematicsStreamingToolboxInputMessage);
      }
   }

   private void sendToolboxStateMessage(ToolboxState toolboxState)
   {
      ToolboxStateMessage toolboxStateMessage = new ToolboxStateMessage();
      toolboxStateMessage.setRequestedToolboxState(toolboxState.toByte());
      toolboxStatePublisher.publish(toolboxStateMessage);
   }

   private List<KinematicsStreamingToolboxMessageSet> loadMessages(InputStream inputStream) throws IOException
   {
      ObjectMapper objectMapper = new ObjectMapper();
      JsonNode jsonNode = objectMapper.readTree(inputStream);
      int size = jsonNode.size();

      List<KinematicsStreamingToolboxMessageSet> allMessages = new ArrayList<>();

      for (int i = 0; i < size; i++)
      {
         JsonNode childNode = jsonNode.get(i);
         KinematicsStreamingToolboxMessageSet messageSet = new KinematicsStreamingToolboxMessageSet(childNode.get(timestampName).asLong());

         if (childNode.has(robotConfigurationDataName))
         {
            messageSet.robotConfigurationData = ROS2Message.createInstance(RobotConfigurationData.class);
            ROS2MessageCdrFileTools.deserializeFromJsonNode(childNode.get(robotConfigurationDataName), messageSet.robotConfigurationData);
         }
         if (childNode.has(capturabilityBasedStatusName))
         {
            messageSet.capturabilityBasedStatus = ROS2Message.createInstance(CapturabilityBasedStatus.class);
            ROS2MessageCdrFileTools.deserializeFromJsonNode(childNode.get(capturabilityBasedStatusName), messageSet.capturabilityBasedStatus);
         }
         if (childNode.has(kinematicsToolboxConfigurationMessageName))
         {
            messageSet.kinematicsToolboxConfigurationMessage = ROS2Message.createInstance(KinematicsToolboxConfigurationMessage.class);
            ROS2MessageCdrFileTools.deserializeFromJsonNode(childNode.get(kinematicsToolboxConfigurationMessageName),
                                                            messageSet.kinematicsToolboxConfigurationMessage);
         }
         if (childNode.has(kinematicsStreamingToolboxInputMessageName))
         {
            messageSet.kinematicsStreamingToolboxInputMessage = ROS2Message.createInstance(KinematicsStreamingToolboxInputMessage.class);
            ROS2MessageCdrFileTools.deserializeFromJsonNode(childNode.get(kinematicsStreamingToolboxInputMessageName),
                                                            messageSet.kinematicsStreamingToolboxInputMessage);
         }

         allMessages.add(messageSet);
      }

      return allMessages;
   }

   public RobotConfigurationData getInitialConfiguration()
   {
      for (int i = 0; i < messages.size(); i++)
      {
         if(messages.get(i).robotConfigurationData != null)
         {
            return messages.get(i).robotConfigurationData;
         }
      }

      return null;
   }

   private class KinematicsStreamingToolboxMessageSet
   {
      private final long timestamp;
      private RobotConfigurationData robotConfigurationData = null;
      private CapturabilityBasedStatus capturabilityBasedStatus = null;
      private KinematicsToolboxConfigurationMessage kinematicsToolboxConfigurationMessage = null;
      private KinematicsStreamingToolboxInputMessage kinematicsStreamingToolboxInputMessage = null;

      KinematicsStreamingToolboxMessageSet(long timestamp)
      {
         this.timestamp = timestamp;
      }
   }

   public static void main(String[] args) throws IOException
   {
      String robotName = "Valkyrie"; // "Atlas"; //

      JFileChooser fileChooser = new JFileChooser();
      fileChooser.setFileFilter(new FileNameExtensionFilter("JSON log", "*.json"));
      int chooserState = fileChooser.showOpenDialog(null);

      if (chooserState == JFileChooser.APPROVE_OPTION)
      {

         InputStream inputStream = new FileInputStream(fileChooser.getSelectedFile());
         new KinematicsStreamingToolboxMessageReplay(robotName, inputStream).replayAllMessages();
      }
   }
}
