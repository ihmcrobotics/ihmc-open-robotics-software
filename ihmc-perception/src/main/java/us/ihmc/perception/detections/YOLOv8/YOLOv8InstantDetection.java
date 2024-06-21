package us.ihmc.perception.detections.YOLOv8;

import perception_msgs.msg.dds.InstantDetectionMessage;
import perception_msgs.msg.dds.YOLOv8NodeMessage;
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

//   public static YOLOv8InstantDetection fromMessage(YOLOv8NodeMessage message)
//   {
//      List<Point3D32> objectPointCloud = new ArrayList<>(message.getObjectPointCloud());
//
//      InstantDetectionMessage instantDetectionMessage = message.getDetectableSceneNode().getInstantDetection();
//      return new YOLOv8InstantDetection(instantDetectionMessage.getDetectedObjectClass().toString(),
//                                        instantDetectionMessage.getDetectedObjectName().toString(),
//                                        instantDetectionMessage.getConfidence(),
//                                        instantDetectionMessage.getObjectPose(),
//                                        MessageTools.toInstant(instantDetectionMessage.getDetectionTime()),
//                                        objectPointCloud);
//   }
}
