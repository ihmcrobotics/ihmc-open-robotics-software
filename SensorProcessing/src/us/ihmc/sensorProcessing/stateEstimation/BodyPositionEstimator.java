package us.ihmc.sensorProcessing.stateEstimation;

import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.robotics.geometry.FramePoint;

public interface BodyPositionEstimator
{
   public abstract void getBodyPosition(FramePoint bodyPositionToPack);
   
   public abstract void getCovariance(Tuple3DBasics covarianceToPack);

   public abstract void estimateBodyPosition();
}
