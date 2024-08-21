package us.ihmc.perception.detections.centerPose;

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

   @Override
   public void toMessage(InstantDetectionMessage message)
   {
      super.toMessage(message);
      for (int i = 0; i < 8; ++i)
      {
         message.getCenterPoseBoundingBoxVertices()[i].set(boundingBoxVertices[i]);
         message.getCenterPoseBoundingBox2dVertices()[i].set(boundingBoxVertices2D[i]);
      }
   }

   public static CenterPoseInstantDetection fromMessage(InstantDetectionMessage message)
   {
      Point3D[] boundingBoxVertices = message.getCenterPoseBoundingBoxVertices();
      Point2D[] boundingBoxVertices2D = Arrays.stream(message.getCenterPoseBoundingBox2dVertices()).map(Point2D::new).toArray(Point2D[]::new);

      return new CenterPoseInstantDetection(message.getDetectedObjectClassAsString(),
                                            message.getDetectedObjectNameAsString(),
                                            message.getConfidence(),
                                            message.getObjectPose(),
                                            MessageTools.toInstant(message.getDetectionTime()),
                                            boundingBoxVertices,
                                            boundingBoxVertices2D);
   }
}
