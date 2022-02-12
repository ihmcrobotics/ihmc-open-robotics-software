package us.ihmc.commonWalkingControlModules.capturePoint.controller;

public class HeuristicICPControllerHelper
{

   public static double computeAdjustmentDistance(double adjustedICP, double firstIntersection, double secondIntersection, double firstPerfect, double secondPerfect, double minICPPushDelta)
   {
//      double maxTowardMiddle = 0.0;
      
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
      
      double midPointOrPerfectBoundaryIfOutside = 0.5 * (firstIntersection + secondIntersection);

      if (midPointOrPerfectBoundaryIfOutside < firstBoundary)
         midPointOrPerfectBoundaryIfOutside = firstBoundary;
      else if (midPointOrPerfectBoundaryIfOutside > secondBoundary)
         midPointOrPerfectBoundaryIfOutside = secondBoundary;
      
      
//      double foo = firstIntersection + maxTowardMiddle;
//      if ((foo > 0.0) && (midPointOrPerfectBoundaryIfOutside > foo))
//         midPointOrPerfectBoundaryIfOutside = foo;
//
//      foo = secondIntersection - maxTowardMiddle;
//      if ((foo < 0.0) && (midPointOrPerfectBoundaryIfOutside < foo))
//         midPointOrPerfectBoundaryIfOutside = foo;
      
      
      double maxAdjustmentBeforeInsideMinICPPush = adjustedICP - minICPPushDelta;
      
      if (midPointOrPerfectBoundaryIfOutside < maxAdjustmentBeforeInsideMinICPPush)
         return midPointOrPerfectBoundaryIfOutside;
      
      else if (maxAdjustmentBeforeInsideMinICPPush > firstIntersection)
         return maxAdjustmentBeforeInsideMinICPPush;
      
      else 
         return firstIntersection;
   }

}
