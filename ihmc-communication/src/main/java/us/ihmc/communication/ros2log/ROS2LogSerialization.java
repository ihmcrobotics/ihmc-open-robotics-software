package us.ihmc.communication.ros2log;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import us.ihmc.idl.serializers.extra.ROS2AbstractSerializer;
import us.ihmc.idl.serializers.extra.ROS2JSONSerializer;
import us.ihmc.idl.serializers.extra.ROS2YAMLSerializer;
import us.ihmc.jros2.ROS2Message;

public enum ROS2LogSerialization
{
   JSON, YAML;

   public ObjectMapper createObjectMapper()
   {
      switch (this)
      {
         case JSON:
            return new ObjectMapper(new JsonFactory());
         case YAML:
            return new ObjectMapper(new YAMLFactory());
         default:
            throw new RuntimeException("Unrecognized serialization entry: " + this);
      }
   }

   @SuppressWarnings({"rawtypes", "unchecked"})
   public <T extends ROS2Message<T>> ROS2AbstractSerializer<T> createSerializer(Class<T> messageClass)
   {
      switch (this)
      {
         case JSON:
            return new ROS2JSONSerializer<>(messageClass);
         case YAML:
            return new ROS2YAMLSerializer<>(messageClass);
         default:
            throw new RuntimeException("Unrecognized serialization entry: " + this);
      }
   }

   public String getFilePostfix()
   {
      return name().toLowerCase();
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
