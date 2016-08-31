package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;

public interface PelvisRotationalStateUpdaterInterface
{

   void initialize();

   void initializeForFrozenState();

   void updateForFrozenState();

   void updateRootJointOrientationAndAngularVelocity();

   void getEstimatedOrientation(FrameOrientation estimatedOrientation);

   void getEstimatedAngularVelocity(FrameVector estimatedAngularVelocityToPack);

}