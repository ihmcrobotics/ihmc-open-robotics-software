package us.ihmc.perception.detections.YOLOv8;

import perception_msgs.msg.dds.InstantDetectionMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.detections.InstantDetection;

import java.time.Instant;
import java.util.ArrayList;
import java.util.List;

public class YOLOv8InstantDetection extends InstantDetection
{
   private final List<Point3D32> objectPointCloud;

   public YOLOv8InstantDetection(String detectedObjectClass, double confidence, Pose3D pose, Instant detectionTime, List<Point3D32> objectPointCloud)
   {
      this(detectedObjectClass, detectedObjectClass, confidence, pose, detectionTime, objectPointCloud);
   }

   public YOLOv8InstantDetection(String detectedObjectClass,
                                 String detectedObjectName,
                                 double confidence,
                                 Pose3D pose,
                                 Instant detectionTime,
                                 List<Point3D32> objectPointCloud)
   {
      super(detectedObjectClass, detectedObjectName, confidence, pose, detectionTime);
      this.objectPointCloud = objectPointCloud;
   }

   public List<Point3D32> getObjectPointCloud()
   {
      return objectPointCloud;
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
   }

   public static YOLOv8InstantDetection fromMessage(InstantDetectionMessage message)
   {
      List<Point3D32> objectPointCloud = new ArrayList<>(message.getYoloObjectPointCloud());

      return new YOLOv8InstantDetection(message.getDetectedObjectClassAsString(),
                                        message.getDetectedObjectNameAsString(),
                                        message.getConfidence(),
                                        message.getObjectPose(),
                                        MessageTools.toInstant(message.getDetectionTime()),
                                        objectPointCloud);
   }
}
