package us.ihmc.commonWalkingControlModules.kinematics;

import javax.vecmath.Tuple3d;

import us.ihmc.utilities.math.geometry.FrameVector;

public interface BodyVelocityEstimator
{
   public abstract void packBodyVelocity(FrameVector bodyVelocityToPack);

   public abstract void packCovariance(Tuple3d covarianceToPack);

   public abstract void estimateBodyVelocity();
}
