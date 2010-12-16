package us.ihmc.commonWalkingControlModules.calculators;

import us.ihmc.utilities.math.geometry.FramePoint2d;

public class EquivalentConstantCoPCalculator
{
   public static FramePoint2d computeEquivalentConstantCoP(FramePoint2d currentCapturePoint, FramePoint2d desiredFinalCapturePoint, double finalTime,
           double comHeight, double gravity)
   {
      currentCapturePoint.checkReferenceFrameMatch(desiredFinalCapturePoint);
      double x = computeEquivalentConstantCoP(currentCapturePoint.getX(), desiredFinalCapturePoint.getX(), finalTime, comHeight, gravity);
      double y = computeEquivalentConstantCoP(currentCapturePoint.getY(), desiredFinalCapturePoint.getY(), finalTime, comHeight, gravity);
      return new FramePoint2d(currentCapturePoint.getReferenceFrame(), x, y);
   }
   
   public static double computeEquivalentConstantCoP(double currentCapturePoint, double desiredFinalCapturePoint, double finalTime, double comHeight, double gravity)
   {
      if (finalTime <= 0.0)
         throw new RuntimeException("finalTime <= 0.0. finalTime = " + finalTime);

      double omega0 = Math.sqrt(gravity / comHeight);
      if (Double.isNaN(omega0))
         throw new RuntimeException("omega0 is NaN. gravity: " + gravity + ", comHeight: " + comHeight);
      
      double exp = Math.exp(omega0 * finalTime);
      return (desiredFinalCapturePoint - currentCapturePoint * exp) / (1.0 - exp);
   }
}
