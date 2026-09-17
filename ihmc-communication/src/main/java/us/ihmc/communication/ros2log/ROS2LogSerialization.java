package us.ihmc.communication.ros2log;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import us.ihmc.idl.serializers.extra.ROS2AbstractSerializer;
import us.ihmc.idl.serializers.extra.ROS2JSONSerializer;
import us.ihmc.idl.serializers.extra.ROS2YAMLSerializer;
import us.ihmc.jros2.ROS2Message;

import java.util.Locale;

public enum ROS2LogSerialization
{
   JSON, YAML;

   public ObjectMapper createObjectMapper()
   {
      return switch (this)
      {
         case JSON -> new ObjectMapper(new JsonFactory());
         case YAML -> new ObjectMapper(new YAMLFactory());
      };
   }

   @SuppressWarnings({"rawtypes", "unchecked"})
   public <T extends ROS2Message<T>> ROS2AbstractSerializer<T> createSerializer(Class<T> messageClass)
   {
      return switch (this)
      {
         case JSON -> new ROS2JSONSerializer<>(messageClass);
         case YAML -> new ROS2YAMLSerializer<>(messageClass);
      };
   }

   public String getFilePostfix()
   {
      return name().toLowerCase(Locale.ROOT);
   }

   public static ROS2LogSerialization fromFileName(String fileName)
   {
      for (ROS2LogSerialization serialization : ROS2LogSerialization.values())
      {
         if (fileName.endsWith(serialization.getFilePostfix()))
            return serialization;
      }

      return null;
   }
}
