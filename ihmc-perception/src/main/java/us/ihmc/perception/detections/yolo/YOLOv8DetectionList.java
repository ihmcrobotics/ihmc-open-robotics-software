package us.ihmc.perception.detections.yolo;

import java.util.ArrayList;

/**
 * {@link ArrayList} of YOLO detections.
 * Helpful for destroying all YOLO detections in the list.
 */
public class YOLOv8DetectionList extends ArrayList<YOLOv8Detection>
{
   public void destroy()
   {
      this.forEach(YOLOv8Detection::destroy);
   }
}
