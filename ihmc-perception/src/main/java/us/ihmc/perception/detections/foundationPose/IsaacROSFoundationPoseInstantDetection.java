package us.ihmc.perception.detections.foundationPose;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.perception.detections.InstantDetection;

import java.time.Instant;

public class IsaacROSFoundationPoseInstantDetection extends InstantDetection
{
   public IsaacROSFoundationPoseInstantDetection(String detectedObjectClass,
                                                 double confidence,
                                                 Pose3DReadOnly pose,
                                                 Instant detectionTime)
   {
      super(detectedObjectClass, confidence, pose, detectionTime);
   }
}
