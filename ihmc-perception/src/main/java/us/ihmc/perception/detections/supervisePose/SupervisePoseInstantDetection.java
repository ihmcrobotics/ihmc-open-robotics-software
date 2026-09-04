package us.ihmc.perception.detections.supervisePose;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.perception.detections.InstantDetection;

import java.time.Instant;

public class SupervisePoseInstantDetection extends InstantDetection
{
   private final SupervisePoseTarget trackedTarget;
   private final Box3DReadOnly boundingBox;

   public SupervisePoseInstantDetection(SupervisePoseTarget trackedTarget,
                                        Box3DReadOnly boundingBox,
                                        Instant detectionTime)
   {
      super(trackedTarget.instance(), 1.0, new Pose3D(boundingBox.getPose()), detectionTime);

      this.trackedTarget = trackedTarget;
      this.boundingBox = boundingBox;
   }

   public SupervisePoseTarget getTrackedTarget()
   {
      return trackedTarget;
   }

   public String getCategory()
   {
      return trackedTarget.category();
   }

   public String getInstance()
   {
      return trackedTarget.instance();
   }

   public String getYoloClass()
   {
      return trackedTarget.yoloClass();
   }

   public Box3DReadOnly getBoundingBox()
   {
      return boundingBox;
   }
}