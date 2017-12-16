package us.ihmc.pathPlanning.visibilityGraphs.tools;

import java.util.List;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class PathTools
{
   public static Point3D getPointAlongPathGivenPercentage(double alpha, List<? extends Point3DReadOnly> path)
   {
      if (path == null || path.isEmpty())
         return null;

      if (path.size() == 1 || alpha <= 0.0)
         return new Point3D(path.get(0));

      if (alpha >= 1.0)
         return new Point3D(path.get(path.size() - 1));

      return getPointAlongPathGivenDistanceFromStart(path, alpha * computePathLength(path));
   }

   public static Point3D getPointAlongPathGivenDistanceFromStart(List<? extends Point3DReadOnly> path, double distance)
   {
      if (path == null || path.isEmpty())
         return null;

      if (path.size() == 1 || distance <= 0.0)
         return new Point3D(path.get(0));

      for (int segmentIndex = 0; segmentIndex < path.size() - 1; segmentIndex++)
      {
         Point3DReadOnly segmentStart = path.get(segmentIndex);
         Point3DReadOnly segmentEnd = path.get(segmentIndex + 1);

         double segmentLength = segmentStart.distance(segmentEnd);

         if (distance <= segmentLength)
         {
            Point3D result = new Point3D();
            result.interpolate(segmentStart, segmentEnd, distance / segmentLength);
            return result;
         }
         else
         {
            distance -= segmentLength;
         }
      }

      // Got here because of numerical error, we are at the path.
      return new Point3D(path.get(path.size() - 1));
   }

   public static double computePathLength(List<? extends Point3DReadOnly> path)
   {
      double pathLength = 0.0;
      for (int i = 1; i < path.size(); i++)
      {
         Point3DReadOnly from = path.get(i - 1);
         Point3DReadOnly to = path.get(i);

         pathLength = pathLength + from.distance(to);
      }
      return pathLength;
   }

   public static double computePathLengthToEnd(List<Point3D> path, double alpha)
   {
      return computePathLength(path) * (1.0 - alpha);
   }

   public static double computePathLengthFromStart(List<? extends Point3DReadOnly> path, double alpha)
   {
      return computePathLength(path) * (alpha);
   }

   public static double computePathLength(List<? extends Point3DReadOnly> path, int startWpIndex, int endWpIndex)
   {
      double pathLength = 0.0;
      for (int i = startWpIndex + 1; i <= endWpIndex; i++)
      {
         pathLength = pathLength + path.get(i - 1).distance(path.get(i));
      }

      return pathLength;
   }
}
