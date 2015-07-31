package us.ihmc.sensorProcessing.stateEstimation;

import javax.vecmath.Tuple3d;

import us.ihmc.robotics.geometry.FrameVector;

public interface BodyVelocityEstimator
{
   public abstract void packBodyVelocity(FrameVector bodyVelocityToPack);

   public abstract void packCovariance(Tuple3d covarianceToPack);

   public abstract void estimateBodyVelocity();
}
