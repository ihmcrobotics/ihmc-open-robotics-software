package us.ihmc.communication.ros2log;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import gnu.trove.list.array.TLongArrayList;
import us.ihmc.jros2.ROS2Message;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.log.LogTools;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;

/**
 * IO helper methods for ROS 2 logging, the log file is timestamped ROS 2 messages with the requested serialization.
 * Timestamps are in ms and t=0 is the first message.
 *
 * File structure is:
 * - MessageClassA
 *    - timestamps
 *       - t0, t1, t2...
 *    - messages
 *       - m0, m1, m2...
 * - MessageClassB
 *    - timestamps
 *       - t0, t1, t2...
 *    - messages
 *       - m0, m1, m2...
 * ...
 */
public class ROS2LogIOTools
{
   public static final String logDirectory = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator + "ros2" + File.separator;

   private static final SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
   public static final String timestampKey = "timestamps";
   public static final String messageKey = "messages";

   static String writeLogFile(List<RecordTopicManager<?>> topicManagers, ROS2LogSerialization serialization)
   {
      try
      {
         Files.createDirectories(Paths.get(new File(logDirectory).getPath()));

         ObjectMapper objectMapper = serialization.createObjectMapper();
         ObjectNode rootNode = objectMapper.createObjectNode();

         long firstTimestamp = Long.MAX_VALUE;
         for (int topic_idx = 0; topic_idx < topicManagers.size(); topic_idx++)
         {
            TLongArrayList timestamps = topicManagers.get(topic_idx).getTimestamps();
            if (timestamps.isEmpty())
               continue;
            firstTimestamp = Math.min(timestamps.get(0), firstTimestamp);
         }
         if (firstTimestamp == Long.MAX_VALUE)
         {
            LogTools.info("Empty ROS 2 log, not writing log file.");
            return null;
         }

         for (int topic_idx = 0; topic_idx < topicManagers.size(); topic_idx++)
         {
            RecordTopicManager<?> topicManager = topicManagers.get(topic_idx);
            TLongArrayList timestampsToLog = topicManager.getTimestamps();
            List<?> messagesToLog = topicManager.getMessages();

            if (messagesToLog.isEmpty())
               continue;

            String topicKey = ROS2LogMessageCodec.topicKeyForMessageClass(topicManager.getTopic().getType());
            ObjectNode topicObject = rootNode.putObject(topicKey);
            ArrayNode timestamps = topicObject.putArray(timestampKey);
            ArrayNode messages = topicObject.putArray(messageKey);

            for (int message_idx = 0; message_idx < messagesToLog.size(); message_idx++)
            {
               timestamps.add(timestampsToLog.get(message_idx) - firstTimestamp);
               @SuppressWarnings({"unchecked", "rawtypes"})
               ROS2Message message = (ROS2Message) messagesToLog.get(message_idx);
               ROS2LogMessageCodec.serializeMessage(serialization, message, messages);
            }
         }

         String fileName = logDirectory + dateFormat.format(new Date()) + "." + serialization.getFilePostfix();
         FileOutputStream outputStream = new FileOutputStream(fileName);
         PrintStream printStream = new PrintStream(outputStream);
         objectMapper.writerWithDefaultPrettyPrinter().writeValue(printStream, rootNode);
         printStream.close();

         LogTools.info("ROS 2 log record finished: " + fileName);
         return fileName;
      }
      catch (Exception e)
      {
         e.printStackTrace();
         return null;
      }
   }

   @SuppressWarnings({"unchecked", "rawtypes"})
   public static List<ReplayTopicManager<?>> loadLogFile(ROS2Node ros2Node, List<ROS2Topic<?>> loggedTopics, File logFile)
   {
      return loadLogFile(logFile, loggedTopics, topic ->
      {
         ROS2Publisher publisher = ros2Node.createPublisher(topic);
         Consumer consumer = message -> publisher.publish((ROS2Message) message);
         return consumer;
      });
   }

   public static List<ReplayTopicManager<?>> loadLogFile(File logFile, List<ROS2Topic<?>> loggedTopics, Function<ROS2Topic, Consumer> messageConsumerGenerator)
   {
      try
      {
         ROS2LogSerialization serialization = ROS2LogSerialization.fromFileName(logFile.getName());
         ObjectMapper objectMapper = serialization.createObjectMapper();
         FileInputStream inputStream = new FileInputStream(logFile);
         List<ReplayTopicManager<?>> topicManagers = new ArrayList<>();

         ObjectNode rootNode = (ObjectNode) objectMapper.readTree(inputStream);
         long firstTimestamp = Long.MAX_VALUE;

         for (int topic_idx = 0; topic_idx < loggedTopics.size(); topic_idx++)
         {
            ROS2Topic<?> topic = loggedTopics.get(topic_idx);
            Consumer<?> messageConsumer = messageConsumerGenerator.apply(topic);
            ReplayTopicManager<?> topicManager = new ReplayTopicManager(topic, messageConsumer);

            Class<? extends ROS2Message<?>> messageClass = topic.getType();
            ObjectNode topicObject = (ObjectNode) ROS2LogMessageCodec.findTopicNode(rootNode, messageClass);
            if (topicObject == null || topicObject.isEmpty())
               continue;

            ArrayNode timestamps = (ArrayNode) topicObject.get(timestampKey);
            ArrayNode messages = (ArrayNode) topicObject.get(messageKey);

            if (timestamps == null || messages == null || timestamps.isEmpty())
               continue;

            if (timestamps.size() != messages.size())
            {
               LogTools.error("Number of timestamps does not match number of messages for {}", messageClass.getName());
               return null;
            }

            for (int message_idx = 0; message_idx < timestamps.size(); message_idx++)
            {
               long timestamp = timestamps.get(message_idx).longValue();
               @SuppressWarnings({"unchecked", "rawtypes"})
               Class messageClassRaw = (Class) messageClass;
               ROS2Message message = ROS2LogMessageCodec.deserializeMessage(serialization, messages.get(message_idx), messageClassRaw);

               topicManager.getTimestamps().add(timestamp);
               @SuppressWarnings({"unchecked", "rawtypes"})
               List messageList = topicManager.getMessages();
               messageList.add(message);
            }

            if (!timestamps.isEmpty())
               firstTimestamp = Math.min(timestamps.get(0).longValue(), firstTimestamp);

            topicManagers.add(topicManager);
         }

         for (int topic_idx = 0; topic_idx < topicManagers.size(); topic_idx++)
         {
            TLongArrayList timestamps = topicManagers.get(topic_idx).getTimestamps();
            for (int message_idx = 0; message_idx < timestamps.size(); message_idx++)
            {
               timestamps.set(message_idx, timestamps.get(message_idx) - firstTimestamp);
            }
         }

         return topicManagers;
      }
      catch (Exception e)
      {
         e.printStackTrace();
         return null;
      }
   }
}
