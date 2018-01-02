package us.ihmc.pathPlanning.visibilityGraphs.tools;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
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

   public static int intersectionBetweenRay2DAndCircle2D(Point2DReadOnly rayOrigin, Vector2DReadOnly rayDirection, Point2DReadOnly circleCenter,
                                                         double circleRadius, Point2DBasics firstIntersectionToPack, Point2DBasics secondIntersectionToPack)
   {
      return intersectionBetweenRay2DAndCircle2D(rayOrigin.getX(), rayOrigin.getY(), rayDirection.getX(), rayDirection.getY(), circleCenter.getX(),
                                                 circleCenter.getY(), circleRadius, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Implementation inspired from:
    * {@link EuclidGeometryTools#intersectionBetweenRay3DAndCylinder3D(double, double, double, us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly, us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly, us.ihmc.euclid.tuple3D.interfaces.Point3DBasics, us.ihmc.euclid.tuple3D.interfaces.Point3DBasics)}
    * 
    * @param rayOriginX
    * @param rayOriginY
    * @param rayDirectionX
    * @param rayDirectionY
    * @param circleCenterX
    * @param circleCenterY
    * @param circleRadius
    * @param firstIntersectionToPack
    * @param secondIntersectionToPack
    * @return
    */
   public static int intersectionBetweenRay2DAndCircle2D(double rayOriginX, double rayOriginY, double rayDirectionX, double rayDirectionY, double circleCenterX,
                                                         double circleCenterY, double circleRadius, Point2DBasics firstIntersectionToPack,
                                                         Point2DBasics secondIntersectionToPack)
   {
      if (circleRadius < 0.0)
         throw new IllegalArgumentException("The circle radius has to be positive");

      double epsilon = 1.0e-12;

      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setToNaN();
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setToNaN();

      double A = EuclidCoreTools.normSquared(rayDirectionX, rayDirectionY);

      if (A < epsilon)
         return 0;

      double startX = rayOriginX - circleCenterX;
      double startY = rayOriginY - circleCenterY;

      double B = 2.0 * (startX * rayDirectionX + startY * rayDirectionY);
      double C = EuclidCoreTools.normSquared(startX, startY) - circleRadius * circleRadius;
      double deltaSquared = B * B - 4 * A * C;

      if (deltaSquared < -epsilon)
         return 0;

      double oneOverTwoA = 0.5 / A;

      if (deltaSquared < epsilon)
      {
         double dCircle = -B / oneOverTwoA;

         if (dCircle < 0.0)
            return 0;

         if (firstIntersectionToPack != null)
         {
            firstIntersectionToPack.set(rayDirectionX, rayDirectionY);
            firstIntersectionToPack.scale(dCircle);
            firstIntersectionToPack.add(startX, startY);
         }
         return 1;
      }
      else
      {
         double delta = Math.sqrt(deltaSquared);
         double dCircle1 = (-B + delta) * oneOverTwoA;
         double dCircle2 = (-B - delta) * oneOverTwoA;

         if (dCircle2 < 0.0)
         {
            dCircle2 = Double.NaN;
         }

         if (dCircle1 < 0.0)
         {
            dCircle1 = dCircle2;
            dCircle2 = Double.NaN;
         }

         if (dCircle2 < dCircle1)
         {
            double temp = dCircle1;
            dCircle1 = dCircle2;
            dCircle2 = temp;
         }

         if (Double.isNaN(dCircle1))
            return 0;

         if (firstIntersectionToPack != null)
         {
            firstIntersectionToPack.set(rayDirectionX, rayDirectionY);
            firstIntersectionToPack.scale(dCircle1);
            firstIntersectionToPack.add(rayOriginX, rayOriginY);
         }

         if (Double.isNaN(dCircle2))
            return 1;

         if (secondIntersectionToPack != null)
         {
            secondIntersectionToPack.set(rayDirectionX, rayDirectionY);
            secondIntersectionToPack.scale(dCircle2);
            secondIntersectionToPack.add(rayOriginX, rayOriginY);
         }

         return 2;
      }
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
