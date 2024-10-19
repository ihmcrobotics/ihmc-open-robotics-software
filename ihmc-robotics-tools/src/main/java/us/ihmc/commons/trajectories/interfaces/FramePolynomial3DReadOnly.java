package us.ihmc.commons.trajectories.interfaces;

import us.ihmc.commons.trajectories.interfaces.FixedFramePositionTrajectoryGenerator;
import us.ihmc.commons.trajectories.interfaces.Polynomial3DReadOnly;
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
