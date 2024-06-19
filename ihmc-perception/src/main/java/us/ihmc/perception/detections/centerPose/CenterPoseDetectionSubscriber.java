package us.ihmc.perception.detections.centerPose;

import perception_msgs.msg.dds.DetectedObjectPacket;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.detections.DetectionManager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.ros2.RealtimeROS2Node;

import java.time.Instant;

public class CenterPoseDetectionSubscriber
{
   public static final RigidBodyTransform CENTERPOSE_DETECTION_TO_IHMC_ZUP_TRANSFORM = new RigidBodyTransform();
   static
   {
      CENTERPOSE_DETECTION_TO_IHMC_ZUP_TRANSFORM.getRotation().setEuler(0.0, Math.toRadians(90.0), Math.toRadians(180.0));
   }

   private final MutableReferenceFrame sensorReferenceFrame = new MutableReferenceFrame();
   private final ReferenceFrame centerPoseOutputFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("CenterPoseOutputFrame",
                                                                                                                          sensorReferenceFrame.getReferenceFrame(),
                                                                                                                          CENTERPOSE_DETECTION_TO_IHMC_ZUP_TRANSFORM);

   private RealtimeROS2Node ros2Node = null;

   private final DetectionManager detectionManager;

   public CenterPoseDetectionSubscriber(DetectionManager detectionManager)
   {
      this.detectionManager = detectionManager;
   }

   public void subscribe()
   {
      if (ros2Node == null)
      {
         ros2Node = ROS2Tools.createRealtimeROS2Node(PubSubImplementation.FAST_RTPS, "center_pose_subscription_node");
         ros2Node.createSubscription(PerceptionAPI.CENTERPOSE_DETECTED_OBJECT, this::receiveDetectedObjectMessage);
         ros2Node.spin();
      }
   }

   public void unsubscribe()
   {
      if (ros2Node != null)
      {
         ros2Node.destroy();
         ros2Node = null;
      }
   }

   private void receiveDetectedObjectMessage(Subscriber<DetectedObjectPacket> subscriber)
   {
      DetectedObjectPacket detectionMessage = subscriber.takeNextData();
      sensorReferenceFrame.getTransformToParent().set(detectionMessage.getSensorPose());
      sensorReferenceFrame.getReferenceFrame().update();

      FramePose3D objectFramePose = new FramePose3D(centerPoseOutputFrame, detectionMessage.getPose());
      objectFramePose.changeFrame(ReferenceFrame.getWorldFrame());

      Pose3D objectPoseInWorld = new Pose3D(objectFramePose);

      // Get bounding box vertices and set to world frame
      Point3D[] boundingBoxVertices = detectionMessage.getBoundingBoxVertices();
      FramePoint3D frameVertex = new FramePoint3D();
      for (Point3D vertex : boundingBoxVertices)
      {
         frameVertex.setIncludingFrame(centerPoseOutputFrame, vertex);
         frameVertex.changeFrame(ReferenceFrame.getWorldFrame());
         vertex.set(frameVertex);
      }

      // Get 2D bounding box in pixel coordinates (z = 0)
      Point2D[] boundingBox2DVertices = new Point2D[detectionMessage.getBoundingBox2dVertices().length];
      for (int i = 0; i < boundingBox2DVertices.length; ++i)
         boundingBox2DVertices[i] = new Point2D(detectionMessage.getBoundingBox2dVertices()[i]);

      CenterPoseInstantDetection newDetection = new CenterPoseInstantDetection(String.valueOf(detectionMessage.getId()),
                                                                               detectionMessage.getObjectTypeAsString(),
                                                                               detectionMessage.getConfidence(),
                                                                               objectPoseInWorld,
                                                                               Instant.now(),
                                                                               boundingBoxVertices,
                                                                               boundingBox2DVertices);

      detectionManager.addDetection(newDetection, CenterPoseInstantDetection.class);
   }
}
