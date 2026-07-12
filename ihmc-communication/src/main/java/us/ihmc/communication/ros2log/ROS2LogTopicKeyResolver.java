package us.ihmc.communication.ros2log;

import com.fasterxml.jackson.databind.JsonNode;

final class ROS2LogTopicKeyResolver
{
   private ROS2LogTopicKeyResolver()
   {
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
