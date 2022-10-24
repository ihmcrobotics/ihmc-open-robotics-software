package us.ihmc.commonWalkingControlModules.capturePoint.controller;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tools.EuclidCoreTools;

public class HeuristicICPControllerHelper
{

   /**
    * Computes how much to adjust the ICP and the CoP based on how a sequence of other points lines up.
    * All values are with respect to the unconstrained CoP, which is at 0.0 here.
    * 
    * @param momentumShiftedICP  Where the ICP is with respect to the unconstrained CoP, but shifted
    *                            based on the vector from perfect CoP to perfect CMP.
    * @param firstIntersection   Where the first foot intersection is along the line from unconstrained
    *                            CoP to adjusted ICP. firstIntersection must be <= secondIntersection.
    * @param secondIntersection  Where the second foot intersection is along the line from
    *                            unconstrained CoP to adjusted ICP. secondIntersection must be >= firstIntersection.
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
   public static double computeAdjustmentDistance(double momentumShiftedICP,
                                                  double firstIntersection,
                                                  double secondIntersection,
                                                  double firstPerfect,
                                                  double secondPerfect,
                                                  double minICPPushDelta,
                                                  double maxProjectionInside)
   {
      // Figure out the two boundaries, based on the perfect boundaries and the intersections.
      double firstBoundary = MathTools.clamp(firstPerfect, firstIntersection, secondIntersection);
      double secondBoundary = MathTools.clamp(secondPerfect, firstIntersection, secondIntersection);

      // For now if you are trying to slow down and you cannot because the unconstrained CoP is in front of the
      // second edge, just project it there.
      if (secondBoundary < 0.0)
         return secondBoundary;

      // Compute the best point to go towards.
      double intersectionMidpoints = 0.5 * (firstIntersection + secondIntersection);
      double midPointOrBestBoundaryIfOutside = MathTools.clamp(intersectionMidpoints, firstBoundary, secondBoundary);

      double maxLocationInside = firstIntersection + maxProjectionInside;
      double maxLocationBeforeICP = momentumShiftedICP - minICPPushDelta;

      double increaseToNoMoreThan = EuclidCoreTools.min(maxLocationInside, midPointOrBestBoundaryIfOutside, maxLocationBeforeICP);
      increaseToNoMoreThan = MathTools.clamp(increaseToNoMoreThan, 0.0, secondIntersection);

      return Math.max(increaseToNoMoreThan, firstIntersection);
   }


}