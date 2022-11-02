package us.ihmc.avatar.stepAdjustment;

import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.robotics.geometry.PlanarRegion;

public interface PlanarRegionSnapperCallback
{
   void advanceFootIndex();

   void footPoseIsOnBoundary();

   void recordSnapTransform(RigidBodyTransformReadOnly snapTransform, PlanarRegion regionSnappedTo);

   void footPoseWasWiggled(RigidBodyTransformReadOnly wiggleTransform);
}
