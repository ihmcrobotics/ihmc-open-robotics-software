package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.robotics.geometry.FrameOrientation;

public interface PelvisRotationalStateUpdaterInterface
{

   void initialize();

   void initializeForFrozenState();

   void updateForFrozenState();

   void updateRootJointOrientationAndAngularVelocity();

   void getEstimatedOrientation(FrameOrientation estimatedOrientation);

   void getEstimatedAngularVelocity(FrameVector3D estimatedAngularVelocityToPack);

}