package us.ihmc.commonWalkingControlModules.calculators;

import us.ihmc.utilities.math.geometry.FramePoint2d;

public class EquivalentConstantCoPCalculator
{
   public static FramePoint2d computeEquivalentConstantCoP(FramePoint2d currentCapturePoint, FramePoint2d desiredFinalCapturePoint, double finalTime,
           double comHeight, double gravity)
   {
      currentCapturePoint.checkReferenceFrameMatch(desiredFinalCapturePoint);
      double omega0 = computeOmega0(comHeight, gravity);
      if (Double.isNaN(omega0))
         throw new RuntimeException("omega0 is NaN. gravity: " + gravity + ", comHeight: " + comHeight);
      double x = computeEquivalentConstantCoP(currentCapturePoint.getX(), desiredFinalCapturePoint.getX(), finalTime, omega0);
      double y = computeEquivalentConstantCoP(currentCapturePoint.getY(), desiredFinalCapturePoint.getY(), finalTime, omega0);

      return new FramePoint2d(currentCapturePoint.getReferenceFrame(), x, y);
   }

   public static double computeEquivalentConstantCoP(double currentCapturePoint, double desiredFinalCapturePoint, double finalTime, double omega0)
   {
      if (finalTime <= 0.0)
         throw new RuntimeException("finalTime <= 0.0. finalTime = " + finalTime);



      double exp = Math.exp(omega0 * finalTime);

      return (desiredFinalCapturePoint - currentCapturePoint * exp) / (1.0 - exp);
   }

   public static FramePoint2d computePredictedICP(FramePoint2d currentCapturePoint, FramePoint2d cop, double time, double comHeight, double gravity)
   {
      currentCapturePoint.checkReferenceFrameMatch(cop);
      double omega0 = computeOmega0(comHeight, gravity);
      if (Double.isNaN(omega0))
         throw new RuntimeException("omega0 is NaN. gravity: " + gravity + ", comHeight: " + comHeight);
      double x = computePredictedCapturePoint(currentCapturePoint.getX(), cop.getX(), time, omega0);
      double y = computePredictedCapturePoint(currentCapturePoint.getY(), cop.getY(), time, omega0);

      return new FramePoint2d(currentCapturePoint.getReferenceFrame(), x, y);
   }

   public static double computePredictedCapturePoint(double currentCapturePoint, double cop, double time, double omega0)
   {
      return (currentCapturePoint - cop) * Math.exp(omega0 * time) + cop;
   }

   private static double computeOmega0(double comHeight, double gravity)
   {
      return Math.sqrt(gravity / comHeight);
   }

}
