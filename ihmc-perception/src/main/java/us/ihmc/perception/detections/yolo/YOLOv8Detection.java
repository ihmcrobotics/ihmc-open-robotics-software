package us.ihmc.perception.detections.yolo;

import org.bytedeco.opencv.opencv_core.Rect;
import us.ihmc.perception.RawImage;
import us.ihmc.tools.Destroyable;

public record YOLOv8Detection(String name, float confidence, Rect boundingBox, RawImage mask) implements Destroyable
{
   public YOLOv8Detection(String name, float confidence, Rect boundingBox, RawImage mask)
   {
      this.name = name;
      this.confidence = confidence;
      this.boundingBox = boundingBox.retainReference();
      this.mask = mask.get();
   }

   @Override
   public Rect boundingBox()
   {
      return new Rect(boundingBox);
   }

   /**
    * Get the mask. {@link RawImage#release()} should be called on the returned mask.
    *
    * @return The object's mask.
    */
   @Override
   public RawImage mask()
   {
      return mask.get();
   }

   @Override
   public void destroy()
   {
      mask.release();
      boundingBox.releaseReference();
   }
}
