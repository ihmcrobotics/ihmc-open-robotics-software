package us.ihmc.perception.detections.yolo;

import perception_msgs.msg.dds.InstantDetectionMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.detections.InstantDetection;

import java.time.Instant;
import java.util.ArrayList;
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
   private final List<Point3D32> objectPointCloud;

   public YOLOv8InstantDetection(String detectedObjectClass,
                                 double confidence,
                                 Pose3DReadOnly currentPoseToCopy,
                                 Instant detectionTime,
                                 RawImage colorImage,
                                 RawImage objectMask,
                                 RawImage depthImage,
                                 List<Point3D32> objectPointCloud)
   {
      this(detectedObjectClass, detectedObjectClass, confidence, currentPoseToCopy, detectionTime, colorImage, objectMask, depthImage, objectPointCloud);
   }

   public YOLOv8InstantDetection(String detectedObjectClass,
                                 String detectedObjectName,
                                 double confidence,
                                 Pose3DReadOnly currentPoseToCopy,
                                 Instant detectionTime,
                                 RawImage colorImage,
                                 RawImage objectMask,
                                 RawImage depthImage,
                                 List<Point3D32> objectPointCloud)
   {
      super(detectedObjectClass, detectedObjectName, confidence, currentPoseToCopy, detectionTime);

      this.colorImage = colorImage.get();
      this.depthImage = depthImage.get();
      this.objectMask = objectMask.get();
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

   @Override
   public void toMessage(InstantDetectionMessage message)
   {
      super.toMessage(message);
      message.getYoloObjectPointCloud().clear();
      for (int i = 0; i < message.getYoloObjectPointCloud().getCurrentCapacity() && i < objectPointCloud.size(); ++i)
      {
         Point3D32 point = message.getYoloObjectPointCloud().add();
         point.set(objectPointCloud.get(i));
      }
      // TODO: Should pack images into message
   }

   public static YOLOv8InstantDetection fromMessage(InstantDetectionMessage message)
   {
      List<Point3D32> objectPointCloud = new ArrayList<>(message.getYoloObjectPointCloud());

      return new YOLOv8InstantDetection(message.getDetectedObjectClassAsString(),
                                        message.getDetectedObjectNameAsString(),
                                        message.getConfidence(),
                                        message.getObjectPose(),
                                        MessageTools.toInstant(message.getDetectionTime()),
                                        RawImage.fromMessage(message.getYoloColorImage()),
                                        RawImage.fromMessage(message.getYoloDepthImage()),
                                        RawImage.fromMessage(message.getYoloObjectMask()),
                                        objectPointCloud);
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
