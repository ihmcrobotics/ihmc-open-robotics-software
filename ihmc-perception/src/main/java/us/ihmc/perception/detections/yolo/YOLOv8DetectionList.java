package us.ihmc.perception.detections.yolo;

import us.ihmc.tools.Destroyable;

import java.util.ArrayList;

public class YOLOv8DetectionList extends ArrayList<YOLOv8Detection> implements Destroyable
{
   @Override
   public void destroy()
   {
      this.forEach(YOLOv8Detection::destroy);
   }
}
