package us.ihmc.robotics.math.trajectories.interfaces;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;

public interface FramePolynomial3DReadOnly extends Polynomial3DReadOnly, FixedFramePositionTrajectoryGenerator
{
   @Override
   FramePoint3DReadOnly getPosition();

   @Override
   FrameVector3DReadOnly getVelocity();

   @Override
   FrameVector3DReadOnly getAcceleration();
}
