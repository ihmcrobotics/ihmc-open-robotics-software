package us.ihmc.sensorProcessing.stateEstimation;

import javax.vecmath.Tuple3d;

import us.ihmc.robotics.geometry.FramePoint;

public interface BodyPositionEstimator
{
   public abstract void getBodyPosition(FramePoint bodyPositionToPack);
   
   public abstract void getCovariance(Tuple3d covarianceToPack);

   public abstract void estimateBodyPosition();
}
