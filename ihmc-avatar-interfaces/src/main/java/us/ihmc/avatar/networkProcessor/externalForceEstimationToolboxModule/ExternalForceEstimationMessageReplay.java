package us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import controller_msgs.msg.dds.*;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

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
   private final RealtimeRos2Node ros2Node;

   private final JSONSerializer<RobotConfigurationData> robotConfigurationDataSerializer = new JSONSerializer<>(new RobotConfigurationDataPubSubType());
   private final JSONSerializer<RobotDesiredConfigurationData> robotDesiredConfigurationDataSerializer = new JSONSerializer<>(new RobotDesiredConfigurationDataPubSubType());

   private final IHMCRealtimeROS2Publisher<RobotConfigurationData> robotConfigurationDataPublisher;
   private final IHMCRealtimeROS2Publisher<RobotDesiredConfigurationData> robotDesiredConfigurationDataPublisher;
   private final IHMCRealtimeROS2Publisher<ToolboxStateMessage> toolboxStatePublisher;
   private final IHMCRealtimeROS2Publisher<ExternalForceEstimationConfigurationMessage> configMessagePublisher;

   public ExternalForceEstimationMessageReplay(String robotName, InputStream inputStream, PubSubImplementation pubSubImplementation) throws IOException
   {
      this.robotName = robotName;
      messages = loadMessages(inputStream);

      String name = getClass().getSimpleName();
      ros2Node = ROS2Tools.createRealtimeRos2Node(pubSubImplementation, "ihmc_" + name);

      MessageTopicNameGenerator controllerPubGenerator = ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
      robotConfigurationDataPublisher = ROS2Tools.createPublisher(ros2Node, RobotConfigurationData.class, controllerPubGenerator);
      robotDesiredConfigurationDataPublisher = ROS2Tools.createPublisher(ros2Node, RobotDesiredConfigurationData.class, controllerPubGenerator);

      MessageTopicNameGenerator toolboxSubTopicNameGenerator = ExternalForceEstimationToolboxModule.getSubscriberTopicNameGenerator(robotName);
      configMessagePublisher = ROS2Tools.createPublisher(ros2Node, ExternalForceEstimationConfigurationMessage.class, toolboxSubTopicNameGenerator);
      toolboxStatePublisher = ROS2Tools.createPublisher(ros2Node, ToolboxStateMessage.class, toolboxSubTopicNameGenerator);

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
