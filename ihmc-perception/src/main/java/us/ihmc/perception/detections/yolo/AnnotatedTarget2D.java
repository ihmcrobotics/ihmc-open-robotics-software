package us.ihmc.perception.detections.yolo;

import us.ihmc.perception.RawImage;

public class AnnotatedTarget2D
{
   public final int targetId;
   public final int trackId;
   public final String name;
   public final float score;
   public final float[] bbox;   // [x1,y1,x2,y2]
   public final RawImage mask;  // retained (may be null)

   public AnnotatedTarget2D(int targetId, int trackId, String name, float score, float[] bbox, RawImage mask)
   {
      this.targetId = targetId;
      this.trackId = trackId;
      this.name = name;
      this.score = score;
      this.bbox = bbox;
      this.mask = mask;
   }

   public void destroy()
   {
      if (mask != null) mask.release();
   }
}