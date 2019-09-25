package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import controller_msgs.msg.dds.*;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.RealtimeRos2Node;

import javax.swing.*;
import javax.swing.filechooser.FileNameExtensionFilter;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;

import static us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxMessageLogger.*;

public class KinematicsStreamingToolboxMessageReplay
{
   private static final PubSubImplementation pubSubImplementation = PubSubImplementation.FAST_RTPS;

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

   public KinematicsStreamingToolboxMessageReplay(String robotName, File file) throws IOException
   {
      messages = loadMessages(file);
      String name = getClass().getSimpleName();

      RealtimeRos2Node ros2Node = ROS2Tools.createRealtimeRos2Node(pubSubImplementation, "ihmc_" + name);

      MessageTopicNameGenerator controllerPubGenerator = ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
      robotConfigurationDataPublisher = ROS2Tools.createPublisher(ros2Node, RobotConfigurationData.class, controllerPubGenerator);
      capturabilityBasedStatusPublisher = ROS2Tools.createPublisher(ros2Node, CapturabilityBasedStatus.class, controllerPubGenerator);

      MessageTopicNameGenerator toolboxSubTopicNameGenerator = KinematicsStreamingToolboxModule.getSubscriberTopicNameGenerator(robotName);
      kinematicsToolboxConfigurationPublisher = ROS2Tools.createPublisher(ros2Node, KinematicsToolboxConfigurationMessage.class, toolboxSubTopicNameGenerator);
      kinematicsStreamingToolboxInputPublisher = ROS2Tools.createPublisher(ros2Node, KinematicsStreamingToolboxInputMessage.class, toolboxSubTopicNameGenerator);
      toolboxStatePublisher = ROS2Tools.createPublisher(ros2Node, ToolboxStateMessage.class, toolboxSubTopicNameGenerator);

      ros2Node.spin();
   }

   public void replayMessages()
   {
      // send wakeup packet
      ToolboxStateMessage toolboxStateMessage = new ToolboxStateMessage();
      toolboxStateMessage.setRequestedToolboxState(ToolboxStateMessage.WAKE_UP);
      toolboxStatePublisher.publish(toolboxStateMessage);
      ThreadTools.sleep(1);

      for (int i = 0; i < messages.size(); i++)
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

         if(i != messages.size() - 1)
         {
            long timeDifferenceNanos = messages.get(i + 1).timestamp - messages.get(i).timestamp;
            ThreadTools.sleep(0, (int) timeDifferenceNanos);
         }
      }

      // send sleep packet
      toolboxStateMessage.setRequestedToolboxState(ToolboxStateMessage.SLEEP);
      toolboxStatePublisher.publish(toolboxStateMessage);
   }

   private List<KinematicsStreamingToolboxMessageSet> loadMessages(File file) throws IOException
   {
      InputStream inputStream = new FileInputStream(file);
      ObjectMapper objectMapper = new ObjectMapper();
      JsonNode jsonNode = objectMapper.readTree(inputStream);
      int size = jsonNode.size();

      List<KinematicsStreamingToolboxMessageSet> allMessages = new ArrayList<>(size);

      for (int i = 0; i < size; i++)
      {
         JsonNode childNode = jsonNode.get(i);
         KinematicsStreamingToolboxMessageSet messageSet = new KinematicsStreamingToolboxMessageSet(jsonNode.get(timestampName).asLong());

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

   public List<KinematicsStreamingToolboxMessageSet> getMessages()
   {
      return messages;
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
         new KinematicsStreamingToolboxMessageReplay(robotName, fileChooser.getSelectedFile());
      }
   }
}
