package us.ihmc.pathPlanning.visibilityGraphs.tools;

import java.util.List;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class PathTools
{
   public static Point3D getPointAlongPath(double alpha, List<Point3D> path)
   {
      //      System.out.println("\n\nComplete Length: " + computePathLength(0.0));
      //      System.out.println("Desired length from the end: " + computePathLength(alpha));

      double pathLength = computePathLength(path);

      double tempPathDist = 0.0;
      double previousBeta = 0.0;

      int upperLimit = -1;
      int lowerLimit = -1;
      for (int i = 1; i < path.size(); i++)
      {
         Point3D from = path.get(i - 1);
         Point3D to = path.get(i);
         tempPathDist = tempPathDist + from.distance(to);

         double beta = tempPathDist / pathLength;

         if (alpha >= previousBeta && alpha <= beta)
         {
            upperLimit = i;
            lowerLimit = i - 1;

            break;
         }

         previousBeta = beta;
      }

      //      System.out.println("It is between " + (lowerLimit) + " and " + upperLimit);

      Vector3D vec = new Vector3D(path.get(upperLimit).getX() - path.get(lowerLimit).getX(), path.get(upperLimit).getY() - path.get(lowerLimit).getY(),
                                  path.get(upperLimit).getZ() - path.get(lowerLimit).getZ());
      vec.normalize();

      double pathFromStart = computePathLengthFromStart(path, alpha);
      //      System.out.println("Path from start: " + pathFromStart);
      //
      double pathFromStartToLowerLimit = computePathLength(path, 0, lowerLimit);
      //      System.out.println("pathFromStartToLowerLimit: " + pathFromStartToLowerLimit);

      double amountOfRelativePath = pathFromStart - pathFromStartToLowerLimit;
      //      System.out.println(amountOfRelativePath + "   " + distanceBetweenLimits);

      Point3D newPoint = new Point3D(path.get(lowerLimit).getX() + vec.getX() * amountOfRelativePath,
                                     path.get(lowerLimit).getY() + vec.getY() * amountOfRelativePath,
                                     path.get(lowerLimit).getZ() + vec.getZ() * amountOfRelativePath);
      return newPoint;
   }

   public static double computePathLength(List<Point3D> path)
   {
      double pathLength = 0.0;
      for (int i = 1; i < path.size(); i++)
      {
         Point3D from = path.get(i - 1);
         Point3D to = path.get(i);

         pathLength = pathLength + from.distance(to);
      }
      return pathLength;
   }

   public static double computePathLength(List<Point3D> path, double alpha)
   {
      return computePathLength(path) * (1.0 - alpha);
   }

   public static double computePathLengthFromStart(List<Point3D> path, double alpha)
   {
      return computePathLength(path) * (alpha);
   }

   public static double computePathLength(List<Point3D> path, int startWpIndex, int endWpIndex)
   {
      double pathLength = 0.0;
      for (int i = startWpIndex + 1; i <= endWpIndex; i++)
      {
         pathLength = pathLength + path.get(i - 1).distance(path.get(i));
      }

      return pathLength;
   }
}
