package us.ihmc.communication.ros2log;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import us.ihmc.communication.serialization.ROS2MessageCdrFileTools;
import us.ihmc.idl.serializers.extra.ROS2AbstractSerializer;
import us.ihmc.jros2.ROS2Message;

import java.io.IOException;

/**
 * Encodes/decodes {@link ROS2Message} instances stored in ROS 2 log files (JSON or YAML arrays).
 * Messages are stored as human-readable JSON/YAML strings (same format as pre-jros2 logs).
 */
final class ROS2LogMessageCodec
{
   private ROS2LogMessageCodec()
   {
   }

   static void serializeMessage(ROS2LogSerialization serialization, ROS2Message<?> message, ArrayNode messagesArray) throws IOException
   {
      @SuppressWarnings({"unchecked", "rawtypes"})
      Class messageClass = message.getClass();
      ROS2AbstractSerializer serializer = serialization.createSerializer(messageClass);
      messagesArray.add(serializer.serializeToString(message));
   }

   static <T extends ROS2Message<T>> T deserializeMessage(ROS2LogSerialization serialization, JsonNode node, Class<T> messageClass) throws IOException
   {
      T message = ROS2Message.createInstance(messageClass);

      if (node == null || node.isNull())
         return message;

      if (node.isTextual())
      {
         String text = node.asText().trim();
         if (text.startsWith("{"))
         {
            ROS2AbstractSerializer<T> serializer = serialization.createSerializer(messageClass);
            return serializer.deserialize(text);
         }

         ROS2MessageCdrFileTools.deserializeFromBase64(text, message);
         return message;
      }

      ROS2AbstractSerializer<T> serializer = serialization.createSerializer(messageClass);
      return serializer.deserialize(node.toString());
   }

   static String topicKeyForMessageClass(Class<?> messageClass)
   {
      return legacyTopicKeyForMessageClass(messageClass);
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

      return rootNode.get(messageClass.getName());
   }
}
