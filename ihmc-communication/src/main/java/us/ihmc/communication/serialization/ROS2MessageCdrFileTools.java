package us.ihmc.communication.serialization;

import us.ihmc.fastddsjava.cdr.CDRBuffer;
import us.ihmc.jros2.ROS2Message;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.TextNode;

import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.util.Base64;

/**
 * Writes and reads {@link ROS2Message} instances as CDR byte arrays (replaces {@code JSONSerializer} + PubSubType for file I/O).
 */
public final class ROS2MessageCdrFileTools
{
   public static <T extends ROS2Message<T>> byte[] serializeToBytes(T message)
   {
      CDRBuffer buffer = new CDRBuffer();
      int payloadSize = message.calculateSizeBytes(0);
      buffer.ensureRemainingCapacity(payloadSize + CDRBuffer.PAYLOAD_HEADER.length + 64);
      buffer.writePayloadHeader();
      message.serialize(buffer);

      ByteBuffer byteBuffer = buffer.getBufferUnsafe();
      byte[] bytes = new byte[byteBuffer.position()];
      byteBuffer.flip();
      byteBuffer.get(bytes);
      return bytes;
   }

   public static <T extends ROS2Message<T>> void deserializeInto(byte[] bytes, T message)
   {
      CDRBuffer buffer = new CDRBuffer();
      buffer.ensureRemainingCapacity(bytes.length);
      buffer.getBufferUnsafe().put(bytes);
      buffer.rewind();
      buffer.readPayloadHeader();
      message.deserialize(buffer);
   }

   public static <T extends ROS2Message<T>> void deserializeInto(InputStream inputStream, T message) throws IOException
   {
      deserializeInto(inputStream.readAllBytes(), message);
   }

   public static boolean isLegacyDdsJson(byte[] bytes)
   {
      return bytes.length > 0 && bytes[0] == '{';
   }

   public static <T extends ROS2Message<T>> String serializeToBase64(T message)
   {
      return Base64.getEncoder().encodeToString(serializeToBytes(message));
   }

   public static <T extends ROS2Message<T>> void deserializeFromBase64(String base64, T message) throws IOException
   {
      byte[] bytes = Base64.getDecoder().decode(base64);
      if (isLegacyDdsJson(bytes))
         throw new IOException("Legacy DDS JSON message format is not supported; re-export logs as CDR base64");
      deserializeInto(bytes, message);
   }

   public static <T extends ROS2Message<T>> JsonNode messageToJsonNode(ObjectMapper objectMapper, T message)
   {
      return TextNode.valueOf(serializeToBase64(message));
   }

   public static <T extends ROS2Message<T>> void deserializeFromJsonNode(JsonNode node, T message) throws IOException
   {
      if (node == null || node.isNull())
         return;
      if (node.isTextual())
      {
         deserializeFromBase64(node.asText(), message);
         return;
      }
      byte[] bytes = node.toString().getBytes();
      if (isLegacyDdsJson(bytes))
         throw new IOException("Legacy DDS JSON message format is not supported; re-export logs as CDR base64");
      deserializeInto(bytes, message);
   }

   private ROS2MessageCdrFileTools()
   {
   }
}
