package us.ihmc.communication.ros2log;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
// import us.ihmc.idl.serializers.extra.AbstractSerializer; // Not available in jros2, commenting out
// import us.ihmc.idl.serializers.extra.JSONSerializer; // Not available in jros2, commenting out
// import us.ihmc.idl.serializers.extra.YAMLSerializer; // Not available in jros2, commenting out
// import us.ihmc.pubsub.TopicDataType; // Not available in jros2, commenting out

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

   // public <T> AbstractSerializer<T> createSerializer(TopicDataType<T> topicDataType) // Not available in jros2
   // {
   //    switch (this)
   //    {
   //       case JSON:
   //          return new JSONSerializer<>(topicDataType);
   //       case YAML:
   //          return new YAMLSerializer<>(topicDataType);
   //       default:
   //          throw new RuntimeException("Unrecognized serialization entry: " + this);
   //    }
   // }

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
