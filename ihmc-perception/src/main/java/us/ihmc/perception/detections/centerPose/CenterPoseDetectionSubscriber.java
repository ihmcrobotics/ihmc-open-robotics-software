package us.ihmc.perception.detections.centerPose;

import perception_msgs.msg.dds.DetectedObjectPacket;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.detections.DetectionManager;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;

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

   private final DetectionManager detectionManager;

   public CenterPoseDetectionSubscriber(ROS2PublishSubscribeAPI ros2, DetectionManager detectionManager)
   {
      this.detectionManager = detectionManager;
      ros2.subscribeViaCallback(PerceptionAPI.CENTERPOSE_DETECTED_OBJECT, this::receiveDetectedObjectMessage);
   }

   private void receiveDetectedObjectMessage(DetectedObjectPacket detectionMessage)
   {
      sensorReferenceFrame.getTransformToParent().set(detectionMessage.getSensorPose());
      sensorReferenceFrame.getReferenceFrame().update();

      FramePose3D objectFramePose = new FramePose3D(centerPoseOutputFrame, detectionMessage.getPose());
      objectFramePose.changeFrame(ReferenceFrame.getWorldFrame());

      Pose3D objectPoseInWorld = new Pose3D(objectFramePose);

      CenterPoseInstantDetection newDetection = new CenterPoseInstantDetection(String.valueOf(detectionMessage.getId()),
                                                                               detectionMessage.getObjectTypeAsString(),
                                                                               detectionMessage.getConfidence(),
                                                                               objectPoseInWorld,
                                                                               Instant.now());

      detectionManager.addDetection(newDetection, CenterPoseInstantDetection.class);
   }
}
