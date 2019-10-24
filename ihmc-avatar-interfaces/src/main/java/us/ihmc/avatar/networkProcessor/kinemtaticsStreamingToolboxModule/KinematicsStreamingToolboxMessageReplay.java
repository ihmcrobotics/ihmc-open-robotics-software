package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import controller_msgs.msg.dds.*;
import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.RealtimeRos2Node;

import javax.swing.*;
import javax.swing.filechooser.FileNameExtensionFilter;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;

import static us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxMessageLogger.*;

/**
 * Loads and replays messages from file. There are two ways to do replay:
 * 1. Calling {@link #replayAllMessages} will immediately stream all messages, using wall time to determine when to send
 * 2. To use as this as a callback based on an external clock, first call {@link #initialize}, then successive calls to {@link #update}
 */
public class KinematicsStreamingToolboxMessageReplay
{
   private final List<KinematicsStreamingToolboxMessageSet> messages;

   private final JSONSerializer<RobotConfigurationData> robotConfigurationDataSerializer = new JSONSerializer<>(new RobotConfigurationDataPubSubType());
   private final JSONSerializer<CapturabilityBasedStatus> capturabilityBasedStatusSerializer = new JSONSerializer<>(new CapturabilityBasedStatusPubSubType());
   private final JSONSerializer<KinematicsToolboxConfigurationMessage> kinematicsToolboxConfigurationMessageSerializer = new JSONSerializer<>(new KinematicsToolboxConfigurationMessagePubSubType());
   private final JSONSerializer<KinematicsStreamingToolboxInputMessage> kinematicsStreamingToolboxInputMessageSerializer = new JSONSerializer<>(new KinematicsStreamingToolboxInputMessagePubSubType());

   private final IHMCRealtimeROS2Publisher<RobotConfigurationData> robotConfigurationDataPublisher;
   private final IHMCRealtimeROS2Publisher<CapturabilityBasedStatus> capturabilityBasedStatusPublisher;
   private final IHMCRealtimeROS2Publisher<KinematicsToolboxConfigurationMessage> kinematicsToolboxConfigurationPublisher;
   private final IHMCRealtimeROS2Publisher<KinematicsStreamingToolboxInputMessage> kinematicsStreamingToolboxInputPublisher;
   private final IHMCRealtimeROS2Publisher<ToolboxStateMessage> toolboxStatePublisher;

   private final MutableInt counter = new MutableInt();
   private double timeOffsetSeconds;

   private final RealtimeRos2Node ros2Node;

   public KinematicsStreamingToolboxMessageReplay(String robotName, InputStream inputStream, PubSubImplementation pubSubImplementation) throws IOException
   {
      messages = loadMessages(inputStream);
      String name = getClass().getSimpleName();

      ros2Node = ROS2Tools.createRealtimeRos2Node(pubSubImplementation, "ihmc_" + name);

      MessageTopicNameGenerator controllerPubGenerator = ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
      robotConfigurationDataPublisher = ROS2Tools.createPublisher(ros2Node, RobotConfigurationData.class, controllerPubGenerator);
      capturabilityBasedStatusPublisher = ROS2Tools.createPublisher(ros2Node, CapturabilityBasedStatus.class, controllerPubGenerator);

      MessageTopicNameGenerator toolboxSubTopicNameGenerator = KinematicsStreamingToolboxModule.getSubscriberTopicNameGenerator(robotName);
      kinematicsToolboxConfigurationPublisher = ROS2Tools.createPublisher(ros2Node, KinematicsToolboxConfigurationMessage.class, toolboxSubTopicNameGenerator);
      kinematicsStreamingToolboxInputPublisher = ROS2Tools.createPublisher(ros2Node, KinematicsStreamingToolboxInputMessage.class, toolboxSubTopicNameGenerator);
      toolboxStatePublisher = ROS2Tools.createPublisher(ros2Node, ToolboxStateMessage.class, toolboxSubTopicNameGenerator);

      ros2Node.spin();
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
      ros2Node.destroy();
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
            messageSet.robotConfigurationData = robotConfigurationDataSerializer.deserialize(childNode.get(robotConfigurationDataName).toString());
         }
         if (childNode.has(capturabilityBasedStatusName))
         {
            messageSet.capturabilityBasedStatus = capturabilityBasedStatusSerializer.deserialize(childNode.get(capturabilityBasedStatusName).toString());
         }
         if (childNode.has(kinematicsToolboxConfigurationMessageName))
         {
            messageSet.kinematicsToolboxConfigurationMessage = kinematicsToolboxConfigurationMessageSerializer
                  .deserialize(childNode.get(kinematicsToolboxConfigurationMessageName).toString());
         }
         if (childNode.has(kinematicsStreamingToolboxInputMessageName))
         {
            messageSet.kinematicsStreamingToolboxInputMessage = kinematicsStreamingToolboxInputMessageSerializer
                  .deserialize(childNode.get(kinematicsStreamingToolboxInputMessageName).toString());
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
         new KinematicsStreamingToolboxMessageReplay(robotName, inputStream, PubSubImplementation.FAST_RTPS).replayAllMessages();
      }
   }
}
