package us.ihmc.perception.detections.yolo;

import org.junit.jupiter.api.Test;

import java.net.URI;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class YOLOv8DetectionExecutorTest
{
   @Test
   public void testParseModelsToLoad()
   {
      assertNull(YOLOv8DetectionExecutor.parseModelsToLoad(null));
      assertNull(YOLOv8DetectionExecutor.parseModelsToLoad(""));
      assertNull(YOLOv8DetectionExecutor.parseModelsToLoad("all"));
      assertTrue(YOLOv8DetectionExecutor.parseModelsToLoad("none").isEmpty());
      assertEquals(Set.of("yolov8n-seg"), YOLOv8DetectionExecutor.parseModelsToLoad("yolov8n-seg"));
      assertEquals(Set.of("yolov8n-seg", "best_multi_04_08_2026"),
                   YOLOv8DetectionExecutor.parseModelsToLoad("yolov8n-seg, best_multi_04_08_2026"));
   }

   @Test
   public void testModelNameFromDirectoryIgnoresTrailingSlash() throws Exception
   {
      assertEquals("yolov8n-seg", YOLOv8DetectionExecutor.modelNameFromDirectory(URI.create("file:/yolo/yolov8n-seg/").toURL()));
      assertEquals("yolov8n-seg", YOLOv8DetectionExecutor.modelNameFromDirectory(URI.create("file:/yolo/yolov8n-seg").toURL()));
   }
}
