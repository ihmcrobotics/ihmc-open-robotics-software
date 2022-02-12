package us.ihmc.commonWalkingControlModules.capturePoint.controller;

public class HeuristicICPControllerHelper
{

   public static double computeAdjustmentDistance(double adjustedICP, double firstIntersection, double secondIntersection, double firstPerfect, double secondPerfect, double minICPPushDelta)
   {
      double firstBoundary = firstPerfect;
      double secondBoundary = secondPerfect;

      if (firstBoundary < firstIntersection)
         firstBoundary = firstIntersection;

      if (secondBoundary < firstIntersection)
         secondBoundary = firstIntersection;

      if (firstBoundary > secondIntersection)
         firstBoundary = secondIntersection;

      if (secondBoundary > secondIntersection)
         secondBoundary = secondIntersection;

      if (secondBoundary < 0.0)
         return secondBoundary;

      double midPointOrPerfectBoundaryIfOutside = 0.5 * (firstIntersection + secondIntersection);

      if (midPointOrPerfectBoundaryIfOutside < firstBoundary)
         midPointOrPerfectBoundaryIfOutside = firstBoundary;
      else if (midPointOrPerfectBoundaryIfOutside > secondBoundary)
         midPointOrPerfectBoundaryIfOutside = secondBoundary;

      // Don't shift towards midpoint if it means speeding up!
      // TODO: Relax this constraint later with some deltas...
      if (midPointOrPerfectBoundaryIfOutside < 0.0)
      {
         midPointOrPerfectBoundaryIfOutside = 0.0;
      }

      double maxAdjustmentBeforeInsideMinICPPush = adjustedICP - minICPPushDelta;

      if (midPointOrPerfectBoundaryIfOutside < maxAdjustmentBeforeInsideMinICPPush)
      {
         return midPointOrPerfectBoundaryIfOutside;
      }

      else if (maxAdjustmentBeforeInsideMinICPPush < 0.0)
      {
         if (firstIntersection > 0.0)
            return firstIntersection;
         else if (secondIntersection < 0.0)
            return secondIntersection;
         else
            return 0.0;
      }

      else if (maxAdjustmentBeforeInsideMinICPPush > firstIntersection)
         return maxAdjustmentBeforeInsideMinICPPush;

      else
         return firstIntersection;
   }

}
