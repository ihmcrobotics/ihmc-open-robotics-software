package us.ihmc.commonWalkingControlModules.calculators;

import us.ihmc.utilities.math.geometry.FramePoint2d;

public class EquivalentConstantCoPCalculator
{
   public static FramePoint2d computeEquivalentConstantCoP(FramePoint2d currentCapturePoint, FramePoint2d desiredFinalCapturePoint, double finalTime,
           double comHeight, double gravity)
   {
      double omega0 = computeOmega0(comHeight, gravity);
      return computeEquivalentConstantCoP(currentCapturePoint, desiredFinalCapturePoint, finalTime, omega0);
   }

   public static FramePoint2d computeEquivalentConstantCoP(FramePoint2d currentCapturePoint, FramePoint2d desiredFinalCapturePoint, double finalTime,
         double omega0)
   {
      currentCapturePoint.checkReferenceFrameMatch(desiredFinalCapturePoint);
      if (Double.isNaN(omega0))
         throw new RuntimeException("omega0 is NaN.");
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
      double omega0 = computeOmega0(comHeight, gravity);
      return computeICPMotionWithConstantCMP(currentCapturePoint, cop, time, omega0);
   }

   /*
    * Note: time can be both positive and negative. 
    * With a positive time, you're computing a future ICP.
    * With a negative time, you're computing a past ICP.
    */
   public static FramePoint2d computeICPMotionWithConstantCMP(FramePoint2d icp, FramePoint2d cmp, double time, double omega0)
   {
      icp.checkReferenceFrameMatch(cmp);
      if (Double.isNaN(omega0))
         throw new RuntimeException("omega0 is NaN");
      double x = computeICPMotionWithConstantCMP(icp.getX(), cmp.getX(), time, omega0);
      double y = computeICPMotionWithConstantCMP(icp.getY(), cmp.getY(), time, omega0);

      return new FramePoint2d(icp.getReferenceFrame(), x, y);
   }
   
   public static double computeICPMotionWithConstantCMP(double currentCapturePoint, double cop, double time, double omega0)
   {
      return (currentCapturePoint - cop) * Math.exp(omega0 * time) + cop;
   }

   private static double computeOmega0(double comHeight, double gravity)
   {
      return Math.sqrt(gravity / comHeight);
   }

   public static double computeMoveTime(FramePoint2d initialDesiredICP, FramePoint2d finalDesiredICP, FramePoint2d equivalentConstantCoP, double comHeight,
         double gravity)
   {
      double omega0 = computeOmega0(comHeight, gravity);
      return computeMoveTime(initialDesiredICP, finalDesiredICP, equivalentConstantCoP, omega0);
   }

   public static double computeMoveTime(FramePoint2d initialDesiredICP, FramePoint2d finalDesiredICP, FramePoint2d equivalentConstantCoP, double omega0)
   {
      double exp = finalDesiredICP.distance(equivalentConstantCoP) / initialDesiredICP.distance(equivalentConstantCoP);
      double ret = Math.log(exp) / omega0;
      return ret;
   }

}
