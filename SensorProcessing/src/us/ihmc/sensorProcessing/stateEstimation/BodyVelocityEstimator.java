package us.ihmc.sensorProcessing.stateEstimation;

import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.robotics.geometry.FrameVector;

public interface BodyVelocityEstimator
{
   public abstract void getBodyVelocity(FrameVector bodyVelocityToPack);

   public abstract void getCovariance(Tuple3DBasics covarianceToPack);

   public abstract void estimateBodyVelocity();
}
