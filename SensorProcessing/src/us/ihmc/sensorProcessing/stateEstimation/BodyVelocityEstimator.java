package us.ihmc.sensorProcessing.stateEstimation;

import javax.vecmath.Tuple3d;

import us.ihmc.robotics.geometry.FrameVector;

public interface BodyVelocityEstimator
{
   public abstract void getBodyVelocity(FrameVector bodyVelocityToPack);

   public abstract void getCovariance(Tuple3d covarianceToPack);

   public abstract void estimateBodyVelocity();
}
