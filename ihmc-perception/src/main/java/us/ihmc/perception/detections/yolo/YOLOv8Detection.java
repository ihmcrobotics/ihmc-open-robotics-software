package us.ihmc.perception.detections.yolo;

import org.bytedeco.opencv.opencv_core.Rect;
import us.ihmc.perception.RawImage;
import us.ihmc.tools.Destroyable;

public record YOLOv8Detection(String objectClass, float confidence, int boundingBoxX, int boundingBoxY, int boundingBoxWidth, int boundingBoxHeight, RawImage mask) implements Destroyable
{
   public YOLOv8Detection(String objectClass, float confidence, Rect boundingBox, RawImage mask)
   {
      this(objectClass, confidence, boundingBox.x(), boundingBox.y(), boundingBox.width(), boundingBox.height(), mask);
   }

   public YOLOv8Detection(String objectClass, float confidence, int boundingBoxX, int boundingBoxY, int boundingBoxWidth, int boundingBoxHeight, RawImage mask)
   {
      this.objectClass = objectClass;
      this.confidence = confidence;
      this.boundingBoxX = boundingBoxX;
      this.boundingBoxY = boundingBoxY;
      this.boundingBoxWidth = boundingBoxWidth;
      this.boundingBoxHeight = boundingBoxHeight;
      this.mask = mask.get();
   }

   /**
    * @return A new {@link Rect} of the bounding box dimensions.
    */
   public Rect boundingBox()
   {
      return new Rect(boundingBoxX, boundingBoxY, boundingBoxWidth, boundingBoxHeight);
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
   }
}
