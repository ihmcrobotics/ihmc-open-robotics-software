package us.ihmc.avatar.stepAdjustment;

import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.robotics.geometry.PlanarRegion;

public interface PlanarRegionSnapperCallback
{
   void reset();

   void setFootIndex(int index);

   void recordUnadjustedFootstep(FramePose3DReadOnly footPose, ConvexPolygon2DReadOnly foothold);

   void recordFootPoseIsOnBoundary();

   void recordSnapTransform(RigidBodyTransformReadOnly snapTransform, PlanarRegion regionSnappedTo);

   void recordWiggleTransform(RigidBodyTransformReadOnly wiggleTransform);
}
