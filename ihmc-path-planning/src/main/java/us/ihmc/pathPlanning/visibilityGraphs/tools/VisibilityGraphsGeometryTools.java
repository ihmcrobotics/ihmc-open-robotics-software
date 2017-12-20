package us.ihmc.pathPlanning.visibilityGraphs.tools;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

/**
 * Sets of geometry tools to be moved to Euclid.
 */
public class VisibilityGraphsGeometryTools
{

   public static boolean doRay2DAndLineSegment2DIntersect(Point2DReadOnly rayOrigin, Vector2DReadOnly rayDirection, Point2DReadOnly lineSegmentStart,
                                                          Point2DReadOnly lineSegmentEnd)
   {
      return doRay2DAndLineSegment2DIntersect(rayOrigin.getX(), rayOrigin.getY(), rayDirection.getX(), rayDirection.getY(), lineSegmentStart.getX(),
                                              lineSegmentStart.getY(), lineSegmentEnd.getX(), lineSegmentEnd.getY());
   }

   public static boolean doRay2DAndLineSegment2DIntersect(double rayOriginX, double rayOriginY, double rayDirectionX, double rayDirectionY,
                                                          double lineSegmentStartX, double lineSegmentStartY, double lineSegmentEndX, double lineSegmentEndY)
   {
      return intersectionBetweenRay2DAndLineSegment2D(rayOriginX, rayOriginY, rayDirectionX, rayDirectionY, lineSegmentStartX, lineSegmentStartY,
                                                      lineSegmentEndX, lineSegmentEndY, null);
   }

   public static Point2D intersectionBetweenRay2DAndLineSegment2D(Point2DReadOnly rayOrigin, Vector2D rayDirection, Point2DReadOnly lineSegmentStart,
                                                                  Point2DReadOnly lineSegmentEnd)
   {
      Point2D intersection = new Point2D();
      boolean success = intersectionBetweenRay2DAndLineSegment2D(rayOrigin.getX(), rayOrigin.getY(), rayDirection.getX(), rayDirection.getY(),
                                                                 lineSegmentStart.getX(), lineSegmentStart.getY(), lineSegmentEnd.getX(), lineSegmentEnd.getY(),
                                                                 intersection);
      if (success)
         return intersection;
      else
         return null;
   }

   public static boolean intersectionBetweenRay2DAndLineSegment2D(Point2DReadOnly rayOrigin, Vector2DReadOnly rayDirection, Point2DReadOnly lineSegmentStart,
                                                                  Point2DReadOnly lineSegmentEnd, Point2DBasics intersectionToPack)
   {
      return intersectionBetweenRay2DAndLineSegment2D(rayOrigin.getX(), rayOrigin.getY(), rayDirection.getX(), rayDirection.getY(), lineSegmentStart.getX(),
                                                      lineSegmentStart.getY(), lineSegmentEnd.getX(), lineSegmentEnd.getY(), intersectionToPack);
   }

   public static boolean intersectionBetweenRay2DAndLineSegment2D(double rayOriginX, double rayOriginY, double rayDirectionX, double rayDirectionY,
                                                                  double lineSegmentStartX, double lineSegmentStartY, double lineSegmentEndX,
                                                                  double lineSegmentEndY, Point2DBasics intersectionToPack)
   {
      double epsilon = 1.0e-7;
   
      double lineSegmentDirectionX = lineSegmentEndX - lineSegmentStartX;
      double lineSegmentDirectionY = lineSegmentEndY - lineSegmentStartY;
   
      double determinant = -rayDirectionX * lineSegmentDirectionY + rayDirectionY * lineSegmentDirectionX;
   
      double dx = lineSegmentStartX - rayOriginX;
      double dy = lineSegmentStartY - rayOriginY;
   
      if (Math.abs(determinant) < epsilon)
      { // The ray and line segment are parallel
        // Check if they are collinear
         double cross = dx * rayDirectionY - dy * rayDirectionX;
         if (Math.abs(cross) < epsilon)
         {
            if (EuclidGeometryTools.isPoint2DInFrontOfRay2D(lineSegmentStartX, lineSegmentStartY, rayOriginX, rayOriginY, rayDirectionX, rayDirectionY))
            {
               if (intersectionToPack != null)
                  intersectionToPack.set(lineSegmentStartX, lineSegmentStartY);
               return true;
            }
   
            if (EuclidGeometryTools.isPoint2DInFrontOfRay2D(lineSegmentEndX, lineSegmentEndY, rayOriginX, rayOriginY, rayDirectionX, rayDirectionY))
            {
               if (intersectionToPack != null)
                  intersectionToPack.set(lineSegmentEndX, lineSegmentEndY);
               return true;
            }
   
            return false;
         }
         // The ray and line segment are parallel but are not collinear, they do not intersect
         else
         {
            return false;
         }
      }
   
      double oneOverDeterminant = 1.0 / determinant;
      double AInverse00 = -lineSegmentDirectionY;
      double AInverse01 = lineSegmentDirectionX;
      double AInverse10 = -rayDirectionY;
      double AInverse11 = rayDirectionX;
   
      double alpha = oneOverDeterminant * (AInverse00 * dx + AInverse01 * dy);
      double beta = oneOverDeterminant * (AInverse10 * dx + AInverse11 * dy);
   
      boolean areIntersecting = alpha > 0.0 - epsilon && 0.0 - epsilon < beta && beta < 1.0 + epsilon;
   
      if (areIntersecting && intersectionToPack != null)
      {
         intersectionToPack.setX(rayOriginX + alpha * rayDirectionX);
         intersectionToPack.setY(rayOriginY + alpha * rayDirectionY);
      }
   
      return areIntersecting;
   }

}
