package us.ihmc.robotDataLogger.logger;

import java.io.File;
import java.io.IOException;

import com.fasterxml.jackson.databind.JsonNode;

import us.ihmc.idl.serializers.extra.CustomDeserializationHandler;
import us.ihmc.idl.serializers.extra.PropertiesSerializer;
import us.ihmc.robotDataLogger.Camera;
import us.ihmc.robotDataLogger.LogProperties;
import us.ihmc.robotDataLogger.LogPropertiesPubSubType;

public class LogPropertiesReader extends LogProperties
{
   public LogPropertiesReader(File file)
   {
      PropertiesSerializer<LogProperties> serializer = new PropertiesSerializer<>(new LogPropertiesPubSubType());
      serializer.setCustomDeserializationHandler(new LegacyFileHandler());
      try
      {
         set(serializer.deserialize(file));
      }
      catch (IOException e)
      {
         throw new RuntimeException("Cannot load properties " + file.getAbsolutePath());
      }
   }

   private static class LegacyFileHandler implements CustomDeserializationHandler<LogProperties>
   {
      @Override
      public void handle(JsonNode node, LogProperties data)
      {
         JsonNode resourceDirectories = node.with("model").get("resourceDirectories");
         if (resourceDirectories != null)
         {
            System.out.println("Handling legacy resource directories");
            for (String directory : resourceDirectories.asText().split(","))
            {
               System.out.println("Adding directory " + directory);
               data.getModel().getResourceDirectoriesList().add(directory);
            }
         }

         JsonNode videoStreams = node.get("videoStreams");
         if (videoStreams != null)
         {
            String[] videos = videoStreams.asText().split(",");
            for (String video : videos)
            {
               System.out.println("Trying " + video + "...");
               JsonNode cameraNode = node.get(video);
               if (cameraNode != null)
               {
                  Camera camera = data.getCameras().add();
                  camera.setName(video);
                  camera.setTimestampFile(cameraNode.path("timestamps").asText());
                  camera.setVideoFile(cameraNode.path("video").asText());
                  camera.setInterlaced(cameraNode.path("interlaced").asBoolean());

                  System.out.println("Added " + camera);
               }
            }
         }
      }
   }

}
