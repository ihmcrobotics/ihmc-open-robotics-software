package us.ihmc.commonWalkingControlModules.calculators;

import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameLine2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;

public class MaximumICPVelocityCalculator
{
   public static double computeMaximumICPVelocity(FrameConvexPolygon2d supportPolygon, FramePoint2d icp, FrameVector2d direction, double omega0)
   {
      icp.changeFrame(supportPolygon.getReferenceFrame());
      direction.changeFrame(supportPolygon.getReferenceFrame());
      
      FrameLine2d lineOfICPMotion = new FrameLine2d(icp, direction);
      FramePoint2d[] intersections = supportPolygon.intersectionWith(lineOfICPMotion);
      
      if (intersections == null)
         return 0.0;
      
      FramePoint2d intersectionToUse = null;
      double minParameterAlongLine = Double.POSITIVE_INFINITY;
      for (int i = 0; i < intersections.length; i++)
      {
         FramePoint2d intersection = intersections[i];
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
