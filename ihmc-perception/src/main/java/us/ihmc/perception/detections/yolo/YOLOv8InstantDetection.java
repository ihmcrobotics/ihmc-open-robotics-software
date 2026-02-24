package us.ihmc.perception.detections.yolo;

import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.detections.InstantDetection;

import java.time.Instant;
import java.util.List;

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
   public float getX1()
   {
      return (float) boundingBox.getMinX();
   }

   @Override
   public float getY1()
   {
      return (float) boundingBox.getMinY();
   }

   @Override
   public float getX2()
   {
      return (float) boundingBox.getMaxX();
   }

   @Override
   public float getY2()
   {
      return (float) boundingBox.getMaxY();
   }

   @Override
   public boolean has3D()
   {
      // In your pipeline pose should always exist, but keep it safe
      return getPose() != null;
   }

   @Override
   public float getCx()
   {
      return (float) getPose().getPosition().getX();
   }

   @Override
   public float getCy()
   {
      return (float) getPose().getPosition().getY();
   }

   @Override
   public float getCz()
   {
      return (float) getPose().getPosition().getZ();
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

   // ---------------- existing YOLO getters ----------------

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