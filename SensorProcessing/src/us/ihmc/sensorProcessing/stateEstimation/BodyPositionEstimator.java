package us.ihmc.sensorProcessing.stateEstimation;

import javax.vecmath.Tuple3d;

import us.ihmc.utilities.math.geometry.FramePoint;

public interface BodyPositionEstimator
{
   public abstract void packBodyPosition(FramePoint bodyPositionToPack);
   
   public abstract void packCovariance(Tuple3d covarianceToPack);

   public abstract void estimateBodyPosition();
}
