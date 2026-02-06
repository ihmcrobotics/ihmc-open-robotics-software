package us.ihmc.perception.detections.foundationPose;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.perception.detections.InstantDetection;

import java.time.Instant;

public class IsaacROSFoundationPoseInstantDetection extends InstantDetection
{
   private final IsaacROSFoundationPoseObject trackedObject;
   private final Box3DReadOnly boundingBox;

   public IsaacROSFoundationPoseInstantDetection(IsaacROSFoundationPoseObject trackedObject,
                                                 Box3DReadOnly boundingBox,
                                                 Instant detectionTime)
   {
      super(trackedObject.meshDirectory, 1.0, new Pose3D(boundingBox.getPose()), detectionTime);

      this.trackedObject = trackedObject;
      this.boundingBox = boundingBox;
   }

   public IsaacROSFoundationPoseObject getObject()
   {
      return trackedObject;
   }

   public Box3DReadOnly getBoundingBox()
   {
      return boundingBox;
   }
}
