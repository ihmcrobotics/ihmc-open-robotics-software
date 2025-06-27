package us.ihmc.perception.detections.foundationPose;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.perception.detections.InstantDetection;

import java.time.Instant;

/**
 * Pose estimate received from FoundationPose at an instant.
 * <p>
 * Name is a bit misleading as FoundationPose estimates object pose, but does not detect objects.
 */
public class FoundationPoseInstantDetection extends InstantDetection
{
   public FoundationPoseInstantDetection(String objectMeshFile, String objectId, Pose3DReadOnly pose, Instant detectionTime)
   {
      super(objectMeshFile, objectId, 1.0, pose, detectionTime);
   }
}