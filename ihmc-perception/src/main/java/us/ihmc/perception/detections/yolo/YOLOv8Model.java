package us.ihmc.perception.detections.yolo;

import org.yaml.snakeyaml.Yaml;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class YOLOv8Model
{
   private final String modelName;
   private final List<String> detectionClassNames = new ArrayList<>();
   private final Path onnxFile;

   public YOLOv8Model(Path modelBaseDirectory)
   {
      if (!YOLOv8Tools.isValidYOLOModelDirectory(modelBaseDirectory))
         throw new IllegalArgumentException("Provided directory is not a YOLO model directory");

      modelName = modelBaseDirectory.getFileName().toString();
      onnxFile = YOLOv8Tools.getONNXFile(modelBaseDirectory);

      // Parse class_names.yaml
      InputStream inputStream = null;
      try
      {
         inputStream = new FileInputStream(YOLOv8Tools.getClassNamesFile(modelBaseDirectory).toFile());
      }
      catch (FileNotFoundException e)
      {
         e.printStackTrace();
      }
      Yaml yaml = new Yaml();
      Map<String, Object> data = yaml.load(inputStream);

      List<String> names = (List<String>) data.get("names");

      detectionClassNames.addAll(names);
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

   public String getObjectClassFromIndex(int i)
   {
      return detectionClassNames.get(i);
   }
}
