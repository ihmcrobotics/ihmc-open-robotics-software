package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import controller_msgs.msg.dds.HandLoadBearingMessage;
import controller_msgs.msg.dds.HandLoadBearingMessagePubSubType;
import controller_msgs.msg.dds.SakeHandDesiredCommandMessage;
import controller_msgs.msg.dds.SakeHandDesiredCommandMessagePubSubType;
import controller_msgs.msg.dds.WholeBodyStreamingMessage;
import controller_msgs.msg.dds.WholeBodyStreamingMessagePubSubType;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import toolbox_msgs.msg.dds.KSTLoggingMessage;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

public class KinematicsStreamingLogger
{
   private static void runStandaloneLogger()
   {
      ROS2Node ros2Node = new ROS2NodeBuilder().build("KinematicsStreamingLogger");

      AtomicBoolean requestStartLogging = new AtomicBoolean();
      AtomicBoolean requestStopLogging = new AtomicBoolean();
      AtomicLong logStartTimeNanos = new AtomicLong();

      AtomicReference<WholeBodyStreamingMessage> latestStreamingMessage = new AtomicReference<>();
      AtomicReference<HandLoadBearingMessage> latestLoadBearingMessage = new AtomicReference<>();
      AtomicReference<SakeHandDesiredCommandMessage> latestSakeHandDesiredCommandMessage = new AtomicReference<>();

      String robotName = "Nadia";

      ROS2Topic<KSTLoggingMessage> loggingTopic = HumanoidControllerAPI.getTopic(KSTLoggingMessage.class, robotName);
      ros2Node.createSubscription(loggingTopic, s ->
      {
         LogTools.info("Received logging request");
         KSTLoggingMessage loggingMessage = s.takeNextData();
         if (loggingMessage.getStartLogging())
         {
            requestStartLogging.set(true);
            logStartTimeNanos.set(System.nanoTime());
         }
         else
         {
            requestStopLogging.set(true);
         }
      });

      ROS2Topic<SakeHandDesiredCommandMessage> objectCarryTopic = HumanoidControllerAPI.getTopic(SakeHandDesiredCommandMessage.class, robotName);
      ros2Node.createSubscription(objectCarryTopic, s ->
      {
         LogTools.info("Received sake message");
         SakeHandDesiredCommandMessage message = s.takeNextData();
         message.setLogTimestamp(System.nanoTime() - logStartTimeNanos.get());
         latestSakeHandDesiredCommandMessage.set(message);
      });

      ROS2Topic<HandLoadBearingMessage> loadBearingTopic = HumanoidControllerAPI.getTopic(HandLoadBearingMessage.class, robotName);
      ros2Node.createSubscription(loadBearingTopic, s ->
      {
         HandLoadBearingMessage message = s.takeNextData();
         message.setLogTimestamp(System.nanoTime() - logStartTimeNanos.get());
         latestLoadBearingMessage.set(message);
      });

      ROS2Topic<WholeBodyStreamingMessage> streamingMessageTopic = HumanoidControllerAPI.getTopic(WholeBodyStreamingMessage.class, robotName);
      ros2Node.createSubscription(streamingMessageTopic, s ->
      {
         WholeBodyStreamingMessage message = s.takeNextData();
         message.setLogTimestamp(System.nanoTime() - logStartTimeNanos.get());
         latestStreamingMessage.set(message);
      });

      AtomicBoolean isLogging = new AtomicBoolean();
      List<WholeBodyStreamingMessage> streamingMessages = new ArrayList<>();
      List<HandLoadBearingMessage> loadBearingMessages = new ArrayList<>();
      List<SakeHandDesiredCommandMessage> SakeHandDesiredCommandMessages = new ArrayList<>();

      while (true)
      {
         if (requestStartLogging.getAndSet(false))
            isLogging.set(true);

         if (isLogging.get())
         {
            WholeBodyStreamingMessage streamingMessage = latestStreamingMessage.getAndSet(null);
            if (streamingMessage != null)
               streamingMessages.add(streamingMessage);

            HandLoadBearingMessage loadBearingMessage = latestLoadBearingMessage.getAndSet(null);
            if (loadBearingMessage != null)
               loadBearingMessages.add(loadBearingMessage);

            SakeHandDesiredCommandMessage SakeHandDesiredCommandMessage = latestSakeHandDesiredCommandMessage.getAndSet(null);
            if (SakeHandDesiredCommandMessage != null)
               SakeHandDesiredCommandMessages.add(SakeHandDesiredCommandMessage);

            if (requestStopLogging.getAndSet(false))
            {
               // export
               export(streamingMessages, loadBearingMessages, SakeHandDesiredCommandMessages);
               isLogging.set(false);
            }
         }
         else
         {
            requestStopLogging.set(false);
         }
      }
   }

   private static void export(List<WholeBodyStreamingMessage> streamingMessages,
                              List<HandLoadBearingMessage> loadBearingMessages,
                              List<SakeHandDesiredCommandMessage> SakeHandDesiredCommandMessages)
   {
      SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
      String logDirectory = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator;

      JSONSerializer<WholeBodyStreamingMessage> wholeBodyStreamingSerializer = new JSONSerializer<>(new WholeBodyStreamingMessagePubSubType());
      JSONSerializer<HandLoadBearingMessage> loadBearingSerializer = new JSONSerializer<>(new HandLoadBearingMessagePubSubType());
      JSONSerializer<SakeHandDesiredCommandMessage> objectCarrySerializer = new JSONSerializer<>(new SakeHandDesiredCommandMessagePubSubType());

      try
      {
         String fileName = logDirectory + dateFormat.format(new Date()) + "_KinematicsStreamingToolbox.json";
         FileOutputStream outputStream = new FileOutputStream(fileName);
         PrintStream printStream = new PrintStream(outputStream);

         JsonFactory jsonFactory = new JsonFactory();
         ObjectMapper objectMapper = new ObjectMapper(jsonFactory);

         ArrayNode root = objectMapper.createArrayNode();

         LogTools.info("Exporting " + streamingMessages.size() + " messages");

         ArrayNode streamingArray = root.addArray();
         for (int i = 0; i < streamingMessages.size(); i++)
         {
            streamingArray.add(objectMapper.readTree(wholeBodyStreamingSerializer.serializeToString(streamingMessages.get(i))));
         }

         ArrayNode loadBearingArray = root.addArray();
         for (int i = 0; i < loadBearingMessages.size(); i++)
         {
            loadBearingArray.add(objectMapper.readTree(loadBearingSerializer.serializeToString(loadBearingMessages.get(i))));
         }

         ArrayNode objectCarryArray = root.addArray();
         for (int i = 0; i < SakeHandDesiredCommandMessages.size(); i++)
         {
            objectCarryArray.add(objectMapper.readTree(objectCarrySerializer.serializeToString(SakeHandDesiredCommandMessages.get(i))));
         }

         objectMapper.writerWithDefaultPrettyPrinter().writeValue(printStream, root);

         printStream.flush();
         outputStream.flush();
         printStream.close();
         outputStream.close();

         streamingMessages.clear();
         loadBearingMessages.clear();
         SakeHandDesiredCommandMessages.clear();
      }
      catch (Exception e)
      {
         LogTools.info("Log unsuccessful");
         e.printStackTrace();
      }
   }

   public static void main(String[] args)
   {
      runStandaloneLogger();
   }
}
