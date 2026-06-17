package us.ihmc.communication.ros2log;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

import java.lang.reflect.Method;
import java.util.Iterator;
import java.util.Map;

final class ROS2LogLegacyEuclidAdapter
{
   private static final String POINT_FIELD = "point";
   private static final String VECTOR_FIELD = "vector";
   private static final String QUATERNION_FIELD = "quaternion";
   private static final String X_FIELD = "x";
   private static final String Y_FIELD = "y";
   private static final String Z_FIELD = "z";
   private static final String W_FIELD = "w";
   private static final String S_FIELD = "s";

   private ROS2LogLegacyEuclidAdapter()
   {
   }

   static String normalizeText(ROS2LogSerialization serialization, String text)
   {
      try
      {
         ObjectMapper objectMapper = serialization.createObjectMapper();
         JsonNode rootNode = objectMapper.readTree(text);
         normalizeWrapperFieldsInPlace(rootNode);
         return objectMapper.writeValueAsString(rootNode);
      }
      catch (Exception ignored)
      {
         return text;
      }
   }

   static void applyValuesSafely(ROS2LogSerialization serialization, Object target, String normalizedText)
   {
      try
      {
         ObjectMapper objectMapper = serialization.createObjectMapper();
         JsonNode rootNode = objectMapper.readTree(normalizedText);
         applyValues(target, rootNode);
      }
      catch (Exception ignored)
      {
      }
   }

   private static void normalizeWrapperFieldsInPlace(JsonNode node)
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
                  if ((POINT_FIELD.equals(fieldName) || VECTOR_FIELD.equals(fieldName)) && parsedTuple.length == 3)
                  {
                     objectNode.set(fieldName, createTupleNode(objectNode, parsedTuple, false));
                     continue;
                  }
                  else if (QUATERNION_FIELD.equals(fieldName) && parsedTuple.length == 4)
                  {
                     objectNode.set(fieldName, createTupleNode(objectNode, parsedTuple, true));
                     continue;
                  }
               }
            }

            normalizeWrapperFieldsInPlace(valueNode);
         }
      }
      else if (node.isArray())
      {
         ArrayNode arrayNode = (ArrayNode) node;
         for (int i = 0; i < arrayNode.size(); i++)
         {
            normalizeWrapperFieldsInPlace(arrayNode.get(i));
         }
      }
   }

   private static ObjectNode createTupleNode(ObjectNode owner, double[] values, boolean isQuaternion)
   {
      ObjectNode tupleNode = owner.objectNode();
      tupleNode.put(X_FIELD, values[0]);
      tupleNode.put(Y_FIELD, values[1]);
      tupleNode.put(Z_FIELD, values[2]);

      if (isQuaternion)
      {
         tupleNode.put(W_FIELD, values[3]);
         tupleNode.put(S_FIELD, values[3]);
      }

      return tupleNode;
   }

   private static void applyValues(Object target, JsonNode node)
   {
      if (target == null || node == null || node.isNull())
         return;

      if (applyPointLikeNode(target, node))
         return;

      if (!node.isObject())
         return;

      ObjectNode objectNode = (ObjectNode) node;
      if (objectNode.size() == 1)
      {
         Map.Entry<String, JsonNode> onlyEntry = objectNode.fields().next();
         if (findGetter(target.getClass(), onlyEntry.getKey()) == null)
         {
            applyValues(target, onlyEntry.getValue());
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
            applyValues(childTarget, field.getValue());
         }
         catch (Exception ignored)
         {
         }
      }
   }

   private static boolean applyPointLikeNode(Object target, JsonNode node)
   {
      double[] point = extractTuple(node, POINT_FIELD, 3);
      if (point == null)
         point = extractTuple(node, VECTOR_FIELD, 3);
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

      double[] quaternion = extractTuple(node, QUATERNION_FIELD, 4);
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

      JsonNode xNode = node.get(X_FIELD);
      JsonNode yNode = node.get(Y_FIELD);
      JsonNode zNode = node.get(Z_FIELD);
      if (xNode == null || yNode == null || zNode == null || !xNode.isNumber() || !yNode.isNumber() || !zNode.isNumber())
         return null;

      if (expectedLength == 3)
      {
         return new double[] {xNode.asDouble(), yNode.asDouble(), zNode.asDouble()};
      }

      JsonNode wNode = node.get(W_FIELD);
      if (wNode == null && quaternion)
         wNode = node.get(S_FIELD);
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
      String getterName = "get" + toJavaMemberName(fieldName);
      try
      {
         return type.getMethod(getterName);
      }
      catch (NoSuchMethodException e)
      {
         return null;
      }
   }

   private static String toJavaMemberName(String fieldName)
   {
      StringBuilder memberName = new StringBuilder();
      boolean capitalizeNext = true;

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
}
