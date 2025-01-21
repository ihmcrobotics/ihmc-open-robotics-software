package us.ihmc.perception.detections.yolo;

import us.ihmc.perception.RawImage;

public class YOLOv8Detection
{
   private RawImage originImage;

   private String className;
   private float confidence;

   private int boundingBoxX;
   private int boundingBoxY;
   private int boundingBoxWidth;
   private int boundingBoxHeight;

   private RawImage mask;
}
