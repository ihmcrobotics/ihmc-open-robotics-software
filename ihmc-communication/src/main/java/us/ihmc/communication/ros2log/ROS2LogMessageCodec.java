package us.ihmc.communication.ros2log;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.communication.serialization.ROS2MessageCdrFileTools;
import us.ihmc.idl.serializers.extra.ROS2AbstractSerializer;
import us.ihmc.jros2.ROS2Message;

import java.io.IOException;
import java.lang.reflect.Method;
import java.util.Iterator;
import java.util.Map;

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
            String normalizedText = normalizeLegacyEuclidWrapperFields(serialization, text);
            ROS2AbstractSerializer<T> serializer = serialization.createSerializer(messageClass);
            T deserialized = serializer.deserialize(normalizedText);

            try
            {
               ObjectMapper objectMapper = serialization.createObjectMapper();
               JsonNode rootNode = objectMapper.readTree(normalizedText);
               applyLegacyEuclidValues(deserialized, rootNode);
            }
            catch (Exception ignored)
            {
            }

            return deserialized;
         }

         ROS2MessageCdrFileTools.deserializeFromBase64(text, message);
         return message;
      }

      ROS2AbstractSerializer<T> serializer = serialization.createSerializer(messageClass);
      return serializer.deserialize(node.toString());
   }

   private static void applyLegacyEuclidValues(Object target, JsonNode node)
   {
      if (target == null || node == null || node.isNull())
         return;

      if (applyPointLikeNode(target, node))
         return;

      if (!node.isObject())
         return;

      ObjectNode objectNode = (ObjectNode) node;

      // Root wrappers commonly use a single key containing the ROS message type.
      if (objectNode.size() == 1)
      {
         Map.Entry<String, JsonNode> onlyEntry = objectNode.fields().next();
         if (findGetter(target.getClass(), onlyEntry.getKey()) == null)
         {
            applyLegacyEuclidValues(target, onlyEntry.getValue());
            return;
         }
      }

      Iterator<Map.Entry<String, JsonNode>> fields = objectNode.fields();
      while (fields.hasNext())
      {
         Map.Entry<String, JsonNode> field = fields.next();
         Method getter = findGetter(target.getClass(), field.getKey());
         if (getter == null)
            continue;

         try
         {
            Object childTarget = getter.invoke(target);
            applyLegacyEuclidValues(childTarget, field.getValue());
         }
         catch (Exception ignored)
         {
         }
      }
   }

   private static String normalizeLegacyEuclidWrapperFields(ROS2LogSerialization serialization, String jsonText)
   {
      try
      {
         ObjectMapper objectMapper = serialization.createObjectMapper();
         JsonNode rootNode = objectMapper.readTree(jsonText);
         normalizeLegacyEuclidWrapperFieldsInPlace(rootNode);
         return objectMapper.writeValueAsString(rootNode);
      }
      catch (Exception ignored)
      {
         return jsonText;
      }
   }

   private static void normalizeLegacyEuclidWrapperFieldsInPlace(JsonNode node)
   {
      if (node == null)
         return;

      if (node.isObject())
      {
         ObjectNode objectNode = (ObjectNode) node;
         Iterator<Map.Entry<String, JsonNode>> fields = objectNode.fields();
         while (fields.hasNext())
         {
            Map.Entry<String, JsonNode> field = fields.next();
            String fieldName = field.getKey();
            JsonNode valueNode = field.getValue();

            if (valueNode != null && valueNode.isTextual())
            {
               double[] parsedTuple = parseLegacyTuple(valueNode.asText());
               if (parsedTuple != null)
               {
                  if ("point".equals(fieldName) || "vector".equals(fieldName))
                  {
                     if (parsedTuple.length == 3)
                     {
                        objectNode.set(fieldName, createTupleNode(objectNode, parsedTuple, false));
                        continue;
                     }
                  }
                  else if ("quaternion".equals(fieldName))
                  {
                     if (parsedTuple.length == 4)
                     {
                        objectNode.set(fieldName, createTupleNode(objectNode, parsedTuple, true));
                        continue;
                     }
                  }
               }
            }

            normalizeLegacyEuclidWrapperFieldsInPlace(valueNode);
         }
      }
      else if (node.isArray())
      {
         ArrayNode arrayNode = (ArrayNode) node;
         for (int i = 0; i < arrayNode.size(); i++)
         {
            normalizeLegacyEuclidWrapperFieldsInPlace(arrayNode.get(i));
         }
      }
   }

   private static ObjectNode createTupleNode(ObjectNode owner, double[] values, boolean isQuaternion)
   {
      ObjectNode tupleNode = owner.objectNode();
      tupleNode.put("x", values[0]);
      tupleNode.put("y", values[1]);
      tupleNode.put("z", values[2]);

      if (isQuaternion)
      {
         tupleNode.put("w", values[3]);
         tupleNode.put("s", values[3]);
      }

      return tupleNode;
   }

   private static boolean applyPointLikeNode(Object target, JsonNode node)
   {
      double[] point = extractTuple(node, "point", 3);
      if (point == null)
         point = extractTuple(node, "vector", 3);
      if (point == null)
         point = extractDirectTuple(node, 3, false);

      if (point != null)
      {
         Object pointTarget = invokeGetter(target, "getPoint");
         if (pointTarget == null)
            pointTarget = invokeGetter(target, "getVector");
         if (pointTarget != null && invokeTupleSetter(pointTarget, point))
            return true;
      }

      double[] quaternion = extractTuple(node, "quaternion", 4);
      if (quaternion == null)
         quaternion = extractDirectTuple(node, 4, true);
      if (quaternion != null)
      {
         Object quaternionTarget = invokeGetter(target, "getQuaternion");
         if (quaternionTarget != null && invokeTupleSetter(quaternionTarget, quaternion))
            return true;
      }

      return false;
   }

   private static double[] extractTuple(JsonNode node, String fieldName, int expectedLength)
   {
      if (!node.isObject())
         return null;

      JsonNode fieldNode = node.get(fieldName);
      if (fieldNode == null || fieldNode.isNull())
         return null;

      if (fieldNode.isTextual())
      {
         double[] values = parseLegacyTuple(fieldNode.asText());
         return values != null && values.length == expectedLength ? values : null;
      }

      return extractDirectTuple(fieldNode, expectedLength, expectedLength == 4);
   }

   private static double[] extractDirectTuple(JsonNode node, int expectedLength, boolean quaternion)
   {
      if (!node.isObject())
         return null;

      JsonNode xNode = node.get("x");
      JsonNode yNode = node.get("y");
      JsonNode zNode = node.get("z");
      if (xNode == null || yNode == null || zNode == null || !xNode.isNumber() || !yNode.isNumber() || !zNode.isNumber())
         return null;

      if (expectedLength == 3)
      {
         return new double[] {xNode.asDouble(), yNode.asDouble(), zNode.asDouble()};
      }

      JsonNode wNode = node.get("w");
      if (wNode == null && quaternion)
         wNode = node.get("s");
      if (wNode == null || !wNode.isNumber())
         return null;

      return new double[] {xNode.asDouble(), yNode.asDouble(), zNode.asDouble(), wNode.asDouble()};
   }

   private static boolean invokeTupleSetter(Object tupleTarget, double[] values)
   {
      if (values.length == 4)
      {
         if (invokeQuaternionComponentSetters(tupleTarget, values))
            return true;

         try
         {
            Method set = tupleTarget.getClass().getMethod("set", double.class, double.class, double.class, double.class);
            set.invoke(tupleTarget, values[0], values[1], values[2], values[3]);
            return true;
         }
         catch (Exception ignored)
         {
         }
      }

      try
      {
         Method set = tupleTarget.getClass().getMethod("set", double.class, double.class, double.class);
         set.invoke(tupleTarget, values[0], values[1], values[2]);
         return true;
      }
      catch (Exception ignored)
      {
      }

      return false;
   }

   private static boolean invokeQuaternionComponentSetters(Object tupleTarget, double[] values)
   {
      // Prefer non-normalizing writes to preserve legacy values exactly.
      try
      {
         Method setUnsafe = tupleTarget.getClass().getMethod("setUnsafe", double.class, double.class, double.class, double.class);
         setUnsafe.invoke(tupleTarget, values[0], values[1], values[2], values[3]);
         return true;
      }
      catch (Exception ignored)
      {
      }

      try
      {
         Method setX = tupleTarget.getClass().getMethod("setX", double.class);
         Method setY = tupleTarget.getClass().getMethod("setY", double.class);
         Method setZ = tupleTarget.getClass().getMethod("setZ", double.class);

         Method setS;
         try
         {
            setS = tupleTarget.getClass().getMethod("setS", double.class);
         }
         catch (NoSuchMethodException noSetS)
         {
            setS = tupleTarget.getClass().getMethod("setW", double.class);
         }

         setX.invoke(tupleTarget, values[0]);
         setY.invoke(tupleTarget, values[1]);
         setZ.invoke(tupleTarget, values[2]);
         setS.invoke(tupleTarget, values[3]);
         return true;
      }
      catch (Exception ignored)
      {
         return false;
      }
   }

   private static Object invokeGetter(Object target, String getterName)
   {
      try
      {
         Method getter = target.getClass().getMethod(getterName);
         return getter.invoke(target);
      }
      catch (Exception ignored)
      {
         return null;
      }
   }

   private static Method findGetter(Class<?> type, String fieldName)
   {
      String getterName = "get" + toJavaMemberName(fieldName, true);
      try
      {
         return type.getMethod(getterName);
      }
      catch (NoSuchMethodException e)
      {
         return null;
      }
   }

   private static String toJavaMemberName(String fieldName, boolean capitalizeFirst)
   {
      StringBuilder memberName = new StringBuilder();
      boolean capitalizeNext = capitalizeFirst;

      for (int i = 0; i < fieldName.length(); i++)
      {
         char character = fieldName.charAt(i);
         if (character == '_')
         {
            capitalizeNext = true;
            continue;
         }

         if (capitalizeNext)
         {
            memberName.append(Character.toUpperCase(character));
            capitalizeNext = false;
         }
         else
         {
            memberName.append(character);
         }
      }

      return memberName.toString();
   }

   private static double[] parseLegacyTuple(String text)
   {
      if (text == null)
         return null;

      String trimmed = text.trim();
      if (trimmed.length() < 2 || trimmed.charAt(0) != '(' || trimmed.charAt(trimmed.length() - 1) != ')')
         return null;

      String[] tokens = trimmed.substring(1, trimmed.length() - 1).split(",");
      if (tokens.length != 3 && tokens.length != 4)
         return null;

      double[] values = new double[tokens.length];
      for (int i = 0; i < tokens.length; i++)
      {
         try
         {
            values[i] = Double.parseDouble(tokens[i].trim());
         }
         catch (NumberFormatException e)
         {
            return null;
         }
      }

      return values;
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
