package us.ihmc.communication.ros2log;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import us.ihmc.communication.serialization.ROS2MessageCdrFileTools;
import us.ihmc.jros2.ROS2Message;

import java.io.IOException;

/**
 * Encodes/decodes {@link ROS2Message} instances stored in ROS 2 log files (JSON or YAML arrays).
 * Messages are stored as base64 CDR payloads via {@link ROS2MessageCdrFileTools}.
 */
final class ROS2LogMessageCodec
{
   private ROS2LogMessageCodec()
   {
   }

   static void serializeMessage(ObjectMapper objectMapper, ROS2Message<?> message, ArrayNode messagesArray)
   {
      @SuppressWarnings({"unchecked", "rawtypes"})
      ROS2Message rawMessage = message;
      messagesArray.add(ROS2MessageCdrFileTools.messageToJsonNode(objectMapper, rawMessage));
   }

   static <T extends ROS2Message<T>> T deserializeMessage(ObjectMapper objectMapper, JsonNode node, Class<T> messageClass) throws IOException
   {
      T message = ROS2Message.createInstance(messageClass);

      if (node == null || node.isNull())
         return message;

      if (node.isTextual())
      {
         String text = node.asText().trim();
         if (text.startsWith("{"))
         {
            throw new IOException("Legacy DDS JSON log entries are not supported; re-record the log with jros2");
         }
         ROS2MessageCdrFileTools.deserializeFromBase64(text, message);
      }
      else
      {
         ROS2MessageCdrFileTools.deserializeFromJsonNode(node, message);
      }

      return message;
   }

   static String topicKeyForMessageClass(Class<?> messageClass)
   {
      return messageClass.getName();
   }

   static String legacyTopicKeyForMessageClass(Class<?> messageClass)
   {
      String name = messageClass.getName();
      int lastDot = name.lastIndexOf('.');
      if (lastDot <= 0)
         return name;
      return name.substring(0, lastDot) + ".msg.dds." + name.substring(lastDot + 1);
   }

   static JsonNode findTopicNode(JsonNode rootNode, Class<?> messageClass)
   {
      JsonNode topicObject = rootNode.get(topicKeyForMessageClass(messageClass));
      if (topicObject != null)
         return topicObject;

      return rootNode.get(legacyTopicKeyForMessageClass(messageClass));
   }
}
