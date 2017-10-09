package us.ihmc.sensorProcessing.stateEstimation;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;

public interface BodyPositionEstimator
{
   public abstract void getBodyPosition(FramePoint3D bodyPositionToPack);
   
   public abstract void getCovariance(Tuple3DBasics covarianceToPack);

   public abstract void estimateBodyPosition();
}
