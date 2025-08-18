package us.ihmc.perception.detections.yolo;

import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.detections.InstantDetection;

import java.time.Instant;
import java.util.List;

/**
 * Extends {@link InstantDetection} by holding onto the depth points that coorespond to
 * the detected segmentation of the object. This has already undergone segmentation erosion
 * and outlier points removed.
 */
public class YOLOv8InstantDetection extends InstantDetection
{
   private final RawImage colorImage;
   private final RawImage depthImage;
   private final RawImage objectMask;
   private final BoundingBox2DReadOnly boundingBox;
   private final List<Point3D32> objectPointCloud;

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
