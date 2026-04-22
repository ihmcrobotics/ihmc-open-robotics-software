package us.ihmc.perception.detections.foundationPose;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.perception.detections.InstantDetection;

import java.time.Instant;

public class CategoryLevelFoundationPoseInstantDetection extends InstantDetection
{
   private final CategoryLevelFoundationPoseTarget trackedTarget;
   private final Box3DReadOnly boundingBox;

   public CategoryLevelFoundationPoseInstantDetection(CategoryLevelFoundationPoseTarget trackedTarget,
                                                      Box3DReadOnly boundingBox,
                                                      Instant detectionTime)
   {
      super(trackedTarget.instance(), 1.0, new Pose3D(boundingBox.getPose()), detectionTime);

      this.trackedTarget = trackedTarget;
      this.boundingBox = boundingBox;
   }

   public CategoryLevelFoundationPoseTarget getTrackedTarget()
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