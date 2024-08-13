package us.ihmc.perception.detections.yolo;

import org.junit.jupiter.api.Test;

public class YOLOv8ObjectDetectorTest
{
   @Test
   public void testInitialization()
   {
      YOLOv8Model model = new YOLOv8Model(YOLOv8Tools.getYOLOModelDirectories().get(0));
      YOLOv8ObjectDetector objectDetector = new YOLOv8ObjectDetector(model);
   }
}
