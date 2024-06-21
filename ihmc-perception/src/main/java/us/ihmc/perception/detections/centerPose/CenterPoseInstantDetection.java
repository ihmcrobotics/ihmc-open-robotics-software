package us.ihmc.perception.detections.centerPose;

import perception_msgs.msg.dds.CenterposeNodeMessage;
import perception_msgs.msg.dds.InstantDetectionMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.detections.InstantDetection;

import java.time.Instant;
import java.util.Arrays;

public class CenterPoseInstantDetection extends InstantDetection
{
   // bounding box vertices in world frame
   private final Point3D[] boundingBoxVertices;
   // bounding box vertices in pixel coordinates of original image sent to CenterPose
   private final Point2D[] boundingBoxVertices2D;

   public CenterPoseInstantDetection(String detectedObjectID,
                                     String detectedObjectCategory,
                                     double confidence,
                                     Pose3D pose,
                                     Instant detectionTime,
                                     Point3D[] boundingBoxVertices,
                                     Point2D[] boundingBoxVertices2D)
   {
      super(detectedObjectID, detectedObjectCategory, confidence, pose, detectionTime);

      this.boundingBoxVertices = boundingBoxVertices;
      this.boundingBoxVertices2D = boundingBoxVertices2D;
   }

   public Point3D[] getBoundingBoxVertices()
   {
      return boundingBoxVertices;
   }

   public Point2D[] getBoundingBoxVertices2D()
   {
      return boundingBoxVertices2D;
   }

//   public static CenterPoseInstantDetection fromMessage(CenterposeNodeMessage message)
//   {
//      Point3D[] boundingBoxVertices = message.getBoundingBoxVertices();
//      Point2D[] boundingBoxVertices2D = Arrays.stream(message.getBoundingBox2dVertices()).map(Point2D::new).toArray(Point2D[]::new);
//
//      InstantDetectionMessage instantDetectionMessage = message.getDetectableSceneNode().getInstantDetection();
//      return new CenterPoseInstantDetection(instantDetectionMessage.getDetectedObjectClassAsString(),
//                                            instantDetectionMessage.getDetectedObjectNameAsString(),
//                                            instantDetectionMessage.getConfidence(),
//                                            instantDetectionMessage.getObjectPose(),
//                                            MessageTools.toInstant(instantDetectionMessage.getDetectionTime()),
//                                            boundingBoxVertices,
//                                            boundingBoxVertices2D);
//   }
}
