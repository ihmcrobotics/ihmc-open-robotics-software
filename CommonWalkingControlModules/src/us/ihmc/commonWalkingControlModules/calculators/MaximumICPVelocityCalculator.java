package us.ihmc.commonWalkingControlModules.calculators;

import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameLine2d;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector2d;

public class MaximumICPVelocityCalculator
{
   public static double computeMaximumICPVelocity(FrameConvexPolygon2d supportPolygon, FramePoint2d icp, FrameVector2d direction, double omega0)
   {
      icp.changeFrame(supportPolygon.getReferenceFrame());
      direction.changeFrame(supportPolygon.getReferenceFrame());
      
      FrameLine2d lineOfICPMotion = new FrameLine2d(icp, direction);
      FramePoint2d[] intersections = supportPolygon.intersectionWith(lineOfICPMotion);
      
      FramePoint2d intersectionToUse = null;
      double minParameterAlongLine = Double.POSITIVE_INFINITY;
      for (FramePoint2d intersection : intersections)
      {
         double parameterAlongLine = lineOfICPMotion.getParameterGivenPointEpsilon(intersection, Double.POSITIVE_INFINITY);
         if (parameterAlongLine < minParameterAlongLine)
         {
            intersectionToUse = intersection;
            minParameterAlongLine = parameterAlongLine;
         }
      }

      return omega0 * icp.distance(intersectionToUse) * Math.signum(-minParameterAlongLine);
   }
}
