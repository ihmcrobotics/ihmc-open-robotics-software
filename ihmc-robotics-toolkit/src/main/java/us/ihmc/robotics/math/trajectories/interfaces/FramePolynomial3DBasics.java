package us.ihmc.robotics.math.trajectories.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameChangeable;

public interface FramePolynomial3DBasics extends FixedFramePolynomial3DBasics, FrameChangeable, FramePositionTrajectoryGenerator
{
   default void setIncludingReferenceFrame(FixedFramePolynomial3DBasics other)
   {
      setReferenceFrame(other.getReferenceFrame());
      FixedFramePolynomial3DBasics.super.set(other);
   }

   default void setIncludingReferenceFrame(ReferenceFrame referenceFrame, Polynomial3DReadOnly other)
   {
      setReferenceFrame(referenceFrame);
      FixedFramePolynomial3DBasics.super.set(referenceFrame, other);
   }
}
