package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;

public interface PelvisRotationalStateUpdaterInterface
{

   void initialize();

   void initializeForFrozenState();

   void updateForFrozenState();

   void updateRootJointOrientationAndAngularVelocity();

   void getEstimatedOrientation(FrameQuaternion estimatedOrientation);

   void getEstimatedAngularVelocity(FrameVector3D estimatedAngularVelocityToPack);

}