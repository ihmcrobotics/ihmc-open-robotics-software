package us.ihmc.perception.detections.yolo;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

public class YOLOv8Model
{
   private final String modelName;
   private final List<String> detectionClassNames = new ArrayList<>();
   private final Path onnxFile;

   public YOLOv8Model(Path modelBaseDirectory)
   {
      if (!YOLOv8Tools.isValidYOLOModelDirectory(modelBaseDirectory))
         throw new IllegalArgumentException("Provided directory is not a YOLO model directory");

      modelName = modelBaseDirectory.getFileName().toString(); // TODO: maybe something other than this?
      onnxFile = YOLOv8Tools.getONNXFile(modelBaseDirectory);

      // TODO: Read class_names.yaml file
   }

   public String getModelName()
   {
      return modelName;
   }

   public byte[] readONNXFile()
   {
      try
      {
         return Files.readAllBytes(onnxFile);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public List<String> getDetectionClassNames()
   {
      return detectionClassNames;
   }
}
