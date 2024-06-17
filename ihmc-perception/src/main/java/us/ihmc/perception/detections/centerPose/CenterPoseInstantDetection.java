package us.ihmc.perception.detections.centerPose;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.perception.detections.InstantDetection;

import java.time.Instant;

public class CenterPoseInstantDetection extends InstantDetection
{
   // TODO: Add CenterPose data stuff

   public CenterPoseInstantDetection(String detectedObjectClass, String detectedObjectName, double confidence, Pose3D pose, Instant detectionTime)
   {
      super(detectedObjectClass, detectedObjectName, confidence, pose, detectionTime);
   }
}
