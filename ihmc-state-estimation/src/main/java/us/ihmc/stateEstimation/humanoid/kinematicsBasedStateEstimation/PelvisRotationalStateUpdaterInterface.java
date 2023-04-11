package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

public interface PelvisRotationalStateUpdaterInterface
{
   void initialize();

   void updateRootJointOrientationAndAngularVelocity();

   FrameOrientation3DReadOnly getEstimatedOrientation();

   FrameVector3DReadOnly getEstimatedAngularVelocity();
}