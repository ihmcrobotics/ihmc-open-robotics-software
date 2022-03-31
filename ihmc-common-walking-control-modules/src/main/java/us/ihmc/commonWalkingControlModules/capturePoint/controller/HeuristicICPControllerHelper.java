package us.ihmc.commonWalkingControlModules.capturePoint.controller;

public class HeuristicICPControllerHelper
{

   /**
    * Computes how much to adjust the ICP and the CoP based on how a sequence of other points lines up.
    * All values are with respect to the unconstrained CoP, which is at 0.0 here.
    * 
    * @param adjustedICP         Where the ICP is with respect to the unconstrained CoP, but adjusted
    *                            based on the vector from perfect CoP to perfect CMP.
    * @param firstIntersection   Where the first foot intersection is along the line from unconstrained
    *                            CoP to adjusted ICP.
    * @param secondIntersection  Where the second foot intersection is along the line from
    *                            unconstrained CoP to adjusted ICP.
    * @param firstPerfect        Where the first perfect boundary is along the line from unconstrained
    *                            CoP to adjusted ICP. Can just be set to the first intersection. Will
    *                            be ignored if before the first intersection.
    * @param secondPerfect       Where the second perfect boundary is along the line from unconstrained
    *                            CoP to adjusted ICP. Can just be set to the second intersection. Will
    *                            be ignored if beyond the second intersection.
    * @param minICPPushDelta     Minimum amount that the ICP must be pushed by the CMP. Adjustment will
    *                            not get any closer than this unless they must in order to get inside
    *                            the foot.
    * @param maxProjectionInside Maximum amount that the CoP may be projected inside the foot, between
    *                            the two boundaries defined by the first and second intersections or
    *                            perfect locations.
    * @return How much to project the unconstrained CoP and unconstrained CMP along the vector from the
    *         unconstrained CMP to the current ICP.
    */
   public static double computeAdjustmentDistance(double adjustedICP,
                                                  double firstIntersection,
                                                  double secondIntersection,
                                                  double firstPerfect,
                                                  double secondPerfect,
                                                  double minICPPushDelta,
                                                  double maxProjectionInside)
   {
      // Figure out the two boundaries, based on the perfect boundaries and the intersections.
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

      double maxLocationInside = firstIntersection + maxProjectionInside;
      double maxLocationBeforeICP = adjustedICP - minICPPushDelta;

      double increaseToNoMoreThan = minimumOf(maxLocationInside, midPointOrPerfectBoundaryIfOutside, maxLocationBeforeICP);
      if (increaseToNoMoreThan < 0.0)
         increaseToNoMoreThan = 0.0;

      if (increaseToNoMoreThan > secondIntersection)
         increaseToNoMoreThan = secondIntersection;

      double increaseToNoLessThan = firstIntersection;

      if (increaseToNoMoreThan > increaseToNoLessThan)
         return increaseToNoMoreThan;

      else
         return increaseToNoLessThan;
   }

   private static final double minimumOf(double a, double b, double c)
   {
      return Math.min(Math.min(a, b), c);
   }

}