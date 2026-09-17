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
      messagesArray.add(serializeToString(serialization, message));
   }

   static <T extends ROS2Message<T>> T deserializeMessage(ROS2LogSerialization serialization, JsonNode node, Class<T> messageClass) throws IOException
   {
      T message = ROS2Message.createInstance(messageClass);

      if (node == null || node.isNull())
         return message;

      ROS2AbstractSerializer<T> serializer = serialization.createSerializer(messageClass);
      if (node.isTextual())
         return deserializeTextNode(serialization, serializer, message, node.asText().trim());

      return serializer.deserialize(node.toString());
   }

   static String topicKeyForMessageClass(Class<?> messageClass)
   {
      return ROS2LogTopicKeyResolver.topicKeyForMessageClass(messageClass);
   }

   static String legacyTopicKeyForMessageClass(Class<?> messageClass)
   {
      return ROS2LogTopicKeyResolver.legacyTopicKeyForMessageClass(messageClass);
   }

   static JsonNode findTopicNode(JsonNode rootNode, Class<?> messageClass)
   {
      return ROS2LogTopicKeyResolver.findTopicNode(rootNode, messageClass);
   }

   private static <T extends ROS2Message<T>> T deserializeTextNode(ROS2LogSerialization serialization,
                                                                    ROS2AbstractSerializer<T> serializer,
                                                                    T emptyMessage,
                                                                    String text) throws IOException
   {
      if (text.startsWith("{"))
         return deserializeJsonText(serialization, serializer, text);

      try
      {
         return serializer.deserialize(text);
      }
      catch (Exception ignored)
      {
         ROS2MessageCdrFileTools.deserializeFromBase64(text, emptyMessage);
         return emptyMessage;
      }
   }

   private static <T extends ROS2Message<T>> T deserializeJsonText(ROS2LogSerialization serialization,
                                                                    ROS2AbstractSerializer<T> serializer,
                                                                    String text) throws IOException
   {
      String normalizedText = ROS2LogLegacyEuclidAdapter.normalizeText(serialization, text);
      T deserialized = serializer.deserialize(normalizedText);
      ROS2LogLegacyEuclidAdapter.applyValuesSafely(serialization, deserialized, normalizedText);
      return deserialized;
   }

   @SuppressWarnings({"unchecked", "rawtypes"})
   private static String serializeToString(ROS2LogSerialization serialization, ROS2Message<?> message) throws IOException
   {
      Class messageClass = message.getClass();
      ROS2AbstractSerializer serializer = serialization.createSerializer(messageClass);
      return serializer.serializeToString(message);
   }
}
