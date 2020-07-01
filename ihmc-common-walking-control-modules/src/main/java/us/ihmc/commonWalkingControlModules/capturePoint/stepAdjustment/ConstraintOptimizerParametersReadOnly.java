package us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment;

public interface ConstraintOptimizerParametersReadOnly
{
   boolean pollParametersChanged();

   double getDesiredDistanceInside();

   double getMaxX();

   double getMaxY();

   boolean getConstrainMaxAdjustment();
}
