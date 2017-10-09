package us.ihmc.sensorProcessing.stateEstimation;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;

public interface BodyVelocityEstimator
{
   public abstract void getBodyVelocity(FrameVector3D bodyVelocityToPack);

   public abstract void getCovariance(Tuple3DBasics covarianceToPack);

   public abstract void estimateBodyVelocity();
}
