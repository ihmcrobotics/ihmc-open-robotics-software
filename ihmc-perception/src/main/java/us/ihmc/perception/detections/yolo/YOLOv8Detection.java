package us.ihmc.perception.detections.yolo;

import org.bytedeco.opencv.opencv_core.Rect;
import us.ihmc.perception.RawImage;
import us.ihmc.euclid.geometry.BoundingBox2D;

public record YOLOv8Detection(String objectClass, int objectClassID, float confidence, int boundingBoxX, int boundingBoxY, int boundingBoxWidth,
                              int boundingBoxHeight, RawImage mask)
{
   public YOLOv8Detection(String objectClass, int objectClassID, float confidence, Rect boundingBox, RawImage mask)
   {
      this(objectClass, objectClassID, confidence, boundingBox.x(), boundingBox.y(), boundingBox.width(), boundingBox.height(), mask);
   }

   public YOLOv8Detection(String objectClass,
                          int objectClassID,
                          float confidence,
                          int boundingBoxX,
                          int boundingBoxY,
                          int boundingBoxWidth,
                          int boundingBoxHeight,
                          RawImage mask)
   {
      this.objectClass = objectClass;
      this.objectClassID = objectClassID;
      this.confidence = confidence;
      this.boundingBoxX = boundingBoxX;
      this.boundingBoxY = boundingBoxY;
      this.boundingBoxWidth = boundingBoxWidth;
      this.boundingBoxHeight = boundingBoxHeight;
      this.mask = mask.get();
   }

   /**
    * @return A new {@link BoundingBox2D} representing the detection's bounding box in pixel coordinates.
    */
   public BoundingBox2D boundingBox()
   {
      return new BoundingBox2D(boundingBoxX, boundingBoxY, boundingBoxX + boundingBoxWidth, boundingBoxY + boundingBoxHeight);
   }

   /**
    * @return A new {@link Rect} of the bounding box dimensions.
    */
   public Rect boundingBoxRect()
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

   public void destroy()
   {
      mask.release();
   }
}
