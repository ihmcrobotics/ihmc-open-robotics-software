package us.ihmc.commonWalkingControlModules.capturePoint.controller;

public class HeuristicICPControllerHelper
{

   public static double computeAdjustmentDistance(double adjustedICP,
                                                  double firstIntersection,
                                                  double secondIntersection,
                                                  double firstPerfect,
                                                  double secondPerfect,
                                                  double minICPPushDelta,
                                                  double maxProjectionInside)
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

      // For now if you are trying to slow down and you cannot because the unconstrained CoP is in front of the
      // second edge, just project it there.
      if (secondBoundary < 0.0)
         return secondBoundary;

      // Compute the best point to go towards.
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
         // Can push as far as the midpoint. But do not push any further inside than maxProjectionInside.

         //TODO: Check the ordering of these. And make test cases where they matter.
         if (midPointOrPerfectBoundaryIfOutside <= 0.0)
            return midPointOrPerfectBoundaryIfOutside;
         else if ((midPointOrPerfectBoundaryIfOutside > 0.0) && (midPointOrPerfectBoundaryIfOutside < firstIntersection + maxProjectionInside))
            return midPointOrPerfectBoundaryIfOutside;
         else if (firstIntersection + maxProjectionInside < 0.0)
            return 0.0;
         else
            return firstIntersection + maxProjectionInside;
      }

      else if (maxAdjustmentBeforeInsideMinICPPush < 0.0)
      {
         //TODO: Check the ordering of these. And make test cases where they matter.

         if (firstIntersection > 0.0)
            return firstIntersection;
         else if (secondIntersection < 0.0)
            return secondIntersection;
         else
            return 0.0;
      }

      else if (maxAdjustmentBeforeInsideMinICPPush > firstIntersection)
      {
         //TODO: Check the ordering of these. And make test cases where they matter.

         if (maxAdjustmentBeforeInsideMinICPPush <= 0.0)
            return maxAdjustmentBeforeInsideMinICPPush;
         else if ((maxAdjustmentBeforeInsideMinICPPush > 0.0) && (maxAdjustmentBeforeInsideMinICPPush < firstIntersection + maxProjectionInside))
            return maxAdjustmentBeforeInsideMinICPPush;
         else if (firstIntersection + maxProjectionInside < 0.0)
            return 0.0;
         else
            return firstIntersection + maxProjectionInside;
      }

      else
         return firstIntersection;
   }

}
