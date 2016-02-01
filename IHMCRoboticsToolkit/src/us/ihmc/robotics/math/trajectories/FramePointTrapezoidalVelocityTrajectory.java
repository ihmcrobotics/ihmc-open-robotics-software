package us.ihmc.robotics.math.trajectories;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.NDoFTrapezoidalVelocityTrajectory.AlphaToAlphaType;


public class FramePointTrapezoidalVelocityTrajectory extends FrameNDoFTrapezoidalVelocityTrajectory
{
   public FramePointTrapezoidalVelocityTrajectory(double t0, FramePoint x0, FramePoint xF, FrameVector v0, FrameVector vF, FrameVector vMax, FrameVector aMax,
           AlphaToAlphaType alphaToAlphaType)
   {
      super(x0.getReferenceFrame(), t0, x0.toArray(), xF.toArray(), v0.toArray(), vF.toArray(), vMax.toArray(), aMax.toArray(), alphaToAlphaType);
      doReferenceFrameChecks(x0, xF, v0, vF, vMax, aMax);
   }

   public FramePoint getPosition(double t)
   {
      return new FramePoint(getReferenceFrame(), getPositionArray(t));
   }

   public FrameVector getVelocity(double t)
   {
      return new FrameVector(getReferenceFrame(), getVelocityArray(t));
   }

   public FrameVector getAcceleration(double t)
   {
      return new FrameVector(getReferenceFrame(), getAccelerationArray(t));
   }

   public FrameVector getMaximumVelocity()
   {
      return new FrameVector(getReferenceFrame(), getMaximumVelocityArray());
   }

   public FrameVector getMaximumAcceleration()
   {
      return new FrameVector(getReferenceFrame(), getMaximumAccelerationArray());
   }

   public FramePoint getInitialPosition()
   {
      return new FramePoint(getReferenceFrame(), getX0Array());
   }

   public FrameVector getInitialVelocity()
   {
      return new FrameVector(getReferenceFrame(), getV0Array());
   }
}
