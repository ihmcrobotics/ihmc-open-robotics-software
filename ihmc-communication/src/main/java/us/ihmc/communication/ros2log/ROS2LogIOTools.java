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
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
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
   public static final String LOG_DIRECTORY = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator + "ros2" + File.separator;
   public static final String TIMESTAMP_KEY = "timestamps";
   public static final String MESSAGE_KEY = "messages";
   private static final DateTimeFormatter LOG_FILE_TIMESTAMP_FORMATTER = DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss");

   // Backward-compatible aliases used by existing tests and callers.
   public static final String logDirectory = LOG_DIRECTORY;
   public static final String timestampKey = TIMESTAMP_KEY;
   public static final String messageKey = MESSAGE_KEY;

   static String writeLogFile(List<RecordTopicManager<?>> topicManagers, ROS2LogSerialization serialization)
   {
      try
      {
         Files.createDirectories(Paths.get(new File(LOG_DIRECTORY).getPath()));

         ObjectMapper objectMapper = serialization.createObjectMapper();
         ObjectNode rootNode = objectMapper.createObjectNode();

         long firstTimestamp = findFirstTimestamp(topicManagers);
         if (firstTimestamp == Long.MAX_VALUE)
         {
            LogTools.info("Empty ROS 2 log, not writing log file.");
            return null;
         }

         for (RecordTopicManager<?> topicManager : topicManagers)
         {
            TLongArrayList timestampsToLog = topicManager.getTimestamps();
            List<?> messagesToLog = topicManager.getMessages();

            if (messagesToLog.isEmpty())
               continue;

            String topicKey = ROS2LogMessageCodec.topicKeyForMessageClass(topicManager.getTopic().getType());
            ObjectNode topicObject = rootNode.putObject(topicKey);
            ArrayNode timestamps = topicObject.putArray(TIMESTAMP_KEY);
            ArrayNode messages = topicObject.putArray(MESSAGE_KEY);

            for (int message_idx = 0; message_idx < messagesToLog.size(); message_idx++)
            {
               timestamps.add(timestampsToLog.get(message_idx) - firstTimestamp);
               serializeUnknownMessage(serialization, messagesToLog.get(message_idx), messages);
            }
         }

         String fileName = LOG_DIRECTORY + LOG_FILE_TIMESTAMP_FORMATTER.format(LocalDateTime.now()) + "." + serialization.getFilePostfix();
         try (FileOutputStream outputStream = new FileOutputStream(fileName); PrintStream printStream = new PrintStream(outputStream))
         {
            objectMapper.writerWithDefaultPrettyPrinter().writeValue(printStream, rootNode);
         }

         LogTools.info("ROS 2 log record finished: " + fileName);
         return fileName;
      }
      catch (Exception e)
      {
         LogTools.error("Failed to write ROS 2 log file", e);
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
         if (serialization == null)
         {
            LogTools.error("Unsupported ROS 2 log serialization for file: {}", logFile.getName());
            return null;
         }

         ObjectMapper objectMapper = serialization.createObjectMapper();
         List<ReplayTopicManager<?>> topicManagers = new ArrayList<>();

         ObjectNode rootNode;
         try (FileInputStream inputStream = new FileInputStream(logFile))
         {
            rootNode = (ObjectNode) objectMapper.readTree(inputStream);
         }

         long firstTimestamp = Long.MAX_VALUE;

         for (ROS2Topic<?> topic : loggedTopics)
         {
            ReplayTopicManager<?> topicManager = loadTopic(serialization, rootNode, topic, messageConsumerGenerator.apply(topic));
            if (topicManager == null)
               continue;

            firstTimestamp = Math.min(topicManager.getTimestamps().get(0), firstTimestamp);
            topicManagers.add(topicManager);
         }

         if (firstTimestamp == Long.MAX_VALUE)
            return topicManagers;

         for (ReplayTopicManager<?> topicManager : topicManagers)
         {
            TLongArrayList timestamps = topicManager.getTimestamps();
            for (int message_idx = 0; message_idx < timestamps.size(); message_idx++)
            {
               timestamps.set(message_idx, timestamps.get(message_idx) - firstTimestamp);
            }
         }

         return topicManagers;
      }
      catch (Exception e)
      {
         LogTools.error("Failed to load ROS 2 log file: " + logFile, e);
         return null;
      }
   }

   private static long findFirstTimestamp(List<RecordTopicManager<?>> topicManagers)
   {
      long firstTimestamp = Long.MAX_VALUE;
      for (RecordTopicManager<?> topicManager : topicManagers)
      {
         TLongArrayList timestamps = topicManager.getTimestamps();
         if (!timestamps.isEmpty())
            firstTimestamp = Math.min(timestamps.get(0), firstTimestamp);
      }
      return firstTimestamp;
   }

   @SuppressWarnings({"rawtypes", "unchecked"})
   private static void serializeUnknownMessage(ROS2LogSerialization serialization, Object messageObject, ArrayNode messages) throws Exception
   {
      ROS2Message message = (ROS2Message) messageObject;
      ROS2LogMessageCodec.serializeMessage(serialization, message, messages);
   }

   @SuppressWarnings({"rawtypes", "unchecked"})
   private static ReplayTopicManager<?> loadTopic(ROS2LogSerialization serialization,
                                                  ObjectNode rootNode,
                                                  ROS2Topic<?> topic,
                                                  Consumer<?> messageConsumer) throws Exception
   {
      Class<? extends ROS2Message<?>> messageClass = topic.getType();
      ObjectNode topicObject = (ObjectNode) ROS2LogMessageCodec.findTopicNode(rootNode, messageClass);
      if (topicObject == null || topicObject.isEmpty())
         return null;

      ArrayNode timestamps = (ArrayNode) topicObject.get(TIMESTAMP_KEY);
      ArrayNode messages = (ArrayNode) topicObject.get(MESSAGE_KEY);
      if (timestamps == null || messages == null || timestamps.isEmpty())
         return null;

      if (timestamps.size() != messages.size())
      {
         LogTools.error("Number of timestamps does not match number of messages for {}", messageClass.getName());
         throw new IllegalStateException("Mismatched message/timestamp counts");
      }

      ReplayTopicManager topicManager = new ReplayTopicManager(topic, messageConsumer);
      Class messageClassRaw = (Class) messageClass;

      for (int messageIndex = 0; messageIndex < timestamps.size(); messageIndex++)
      {
         long timestamp = timestamps.get(messageIndex).longValue();
         ROS2Message<?> message = ROS2LogMessageCodec.deserializeMessage(serialization, messages.get(messageIndex), messageClassRaw);
         topicManager.getTimestamps().add(timestamp);
         topicManager.getMessages().add(message);
      }

      return topicManager;
   }
}
