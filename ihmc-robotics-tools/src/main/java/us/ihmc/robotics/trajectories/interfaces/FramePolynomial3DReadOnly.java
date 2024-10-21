package us.ihmc.robotics.trajectories.interfaces;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

public interface FramePolynomial3DReadOnly extends Polynomial3DReadOnly, FixedFramePositionTrajectoryGenerator
{
   @Override
   FramePoint3DReadOnly getPosition();

   @Override
   FrameVector3DReadOnly getVelocity();

   @Override
   FrameVector3DReadOnly getAcceleration();
}
