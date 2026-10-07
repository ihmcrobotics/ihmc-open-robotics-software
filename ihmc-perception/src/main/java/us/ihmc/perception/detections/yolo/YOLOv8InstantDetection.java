package us.ihmc.perception.detections.yolo;

import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.detections.InstantDetection;

import java.time.Instant;
import java.util.List;

/**
 * YOLO detection with an image bounding box and a depth-derived centroid position.
 * The YOLO executors supply identity orientation; they do not estimate object orientation.
 */
public class YOLOv8InstantDetection extends InstantDetection implements TrackableDetection
{
   private final RawImage colorImage;
   private final RawImage depthImage;
   private final RawImage objectMask;
   private final BoundingBox2DReadOnly boundingBox;
   private final List<Point3D32> objectPointCloud;

   private int trackId = -1;

   public YOLOv8InstantDetection(String detectedObjectClass,
                                 double confidence,
                                 Pose3DReadOnly pose,
                                 Instant detectionTime,
                                 RawImage colorImage,
                                 RawImage objectMask,
                                 RawImage depthImage,
                                 BoundingBox2DReadOnly boundingBox,
                                 List<Point3D32> objectPointCloud)
   {
      super(detectedObjectClass, detectedObjectClass, confidence, pose, detectionTime);

      this.colorImage = colorImage.get();
      this.depthImage = depthImage.get();
      this.objectMask = objectMask.get();
      this.boundingBox = boundingBox;
      this.objectPointCloud = objectPointCloud;
   }

   // ---------------- TrackableDetection ----------------

   @Override
   public String getObjectClass()
   {
      return getDetectedObjectClass();
   }

   @Override
   public double getConfidence()
   {
      return super.getConfidence();
   }

   @Override
   public int getTrackId()
   {
      return trackId;
   }

   @Override
   public void setTrackId(int trackId)
   {
      this.trackId = trackId;
   }

   public List<Point3D32> getObjectPointCloud()
   {
      return objectPointCloud;
   }

   public RawImage getColorImage()
   {
      return colorImage;
   }

   public RawImage getDepthImage()
   {
      return depthImage;
   }

   public RawImage getObjectMask()
   {
      return objectMask;
   }

   @Override
   public BoundingBox2DReadOnly getBoundingBox()
   {
      return boundingBox;
   }

   @Override
   public void destroy()
   {
      super.destroy();
      colorImage.release();
      depthImage.release();
      objectMask.release();
   }
}
