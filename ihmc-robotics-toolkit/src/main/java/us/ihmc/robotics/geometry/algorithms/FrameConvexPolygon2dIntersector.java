package us.ihmc.robotics.geometry.algorithms;

import org.apache.commons.math3.util.Pair;

import us.ihmc.commons.Epsilons;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FrameLine3D;
import us.ihmc.euclid.referenceFrame.FrameLineSegment3D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.robotics.geometry.shapes.FramePlane3d;

/**
 * Theoretically finished but not unit tested or bug free. Have to abandon this
 * now but we might get around to needing it at some point in the future.
 * 
 * @author dcalvert
 */
public class FrameConvexPolygon2dIntersector
{
   private final FramePlane3d planeOne;
   private final FramePlane3d planeTwo;
   private final FrameLine3D intersectionOfPlanes;
   private final FrameLine2D planeIntersectionOnPolygonPlane;
   private final Pair<FramePoint2D, FramePoint2D> lineIntersectionOnPolygonPlane;
   private final Pair<FramePoint3D, FramePoint3D> intersectionWithPolygonOne;
   private final Pair<FramePoint3D, FramePoint3D> intersectionWithPolygonTwo;
   private final Vector3D point2Vector;
   private final Vector3D point3Vector;
   private final Vector3D point4Vector;
   private boolean noIntersection;

   public FrameConvexPolygon2dIntersector()
   {
      planeOne = new FramePlane3d();
      planeTwo = new FramePlane3d();
      intersectionOfPlanes = new FrameLine3D();
      planeIntersectionOnPolygonPlane = new FrameLine2D();
      lineIntersectionOnPolygonPlane = new Pair<FramePoint2D, FramePoint2D>(new FramePoint2D(), new FramePoint2D());
      intersectionWithPolygonOne = new Pair<FramePoint3D, FramePoint3D>(new FramePoint3D(), new FramePoint3D());
      intersectionWithPolygonTwo = new Pair<FramePoint3D, FramePoint3D>(new FramePoint3D(), new FramePoint3D());
      point2Vector = new Vector3D();
      point3Vector = new Vector3D();
      point4Vector = new Vector3D();
      noIntersection = false;
   }

   public void intersect3d(FrameConvexPolygon2D polygonOne, FrameConvexPolygon2D polygonTwo, FrameLineSegment3D intersectionToPack)
   {
      noIntersection = false;

      planeOne.setIncludingFrame(polygonOne.getReferenceFrame(), 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
      planeTwo.setIncludingFrame(polygonTwo.getReferenceFrame(), 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
      planeTwo.changeFrame(planeOne.getReferenceFrame());

      // if planes are parallel, a 2d result
      //      if (planeOne.getPoint(pointToPack);)

      intersectTwoPlanes(planeOne, planeTwo, intersectionOfPlanes);

      if (intersectionOfPlanes.containsNaN())
      {
         intersectionToPack.setToNaN();
         return;
      }

      intersectASinglePolygon(polygonOne, intersectionWithPolygonOne);

      if (noIntersection)
      {
         intersectionToPack.setToNaN();
         return;
      }

      intersectASinglePolygon(polygonTwo, intersectionWithPolygonTwo);

      if (noIntersection)
      {
         intersectionToPack.setToNaN();
         return;
      }

      intersectionWithPolygonOne.getFirst().changeFrame(ReferenceFrame.getWorldFrame());
      intersectionWithPolygonOne.getSecond().changeFrame(ReferenceFrame.getWorldFrame());
      intersectionWithPolygonTwo.getFirst().changeFrame(ReferenceFrame.getWorldFrame());
      intersectionWithPolygonTwo.getSecond().changeFrame(ReferenceFrame.getWorldFrame());
      findIntersectionAmongPoints(intersectionWithPolygonOne.getFirst(), intersectionWithPolygonOne.getSecond(), intersectionWithPolygonTwo.getFirst(),
                                  intersectionWithPolygonTwo.getSecond(), intersectionToPack);

      if (noIntersection)
      {
         intersectionToPack.setToNaN();
         return;
      }
   }

   private void findIntersectionAmongPoints(FramePoint3D point1, FramePoint3D point2, FramePoint3D point3, FramePoint3D point4, FrameLineSegment3D intersectionToPack)
   {
      if (allAreValid(point1, point2, point3, point4))
      {
         point2Vector.sub(point2, point1);
         point3Vector.sub(point3, point1);
         point4Vector.sub(point4, point1);

         double oneToTwoSqaured = point2Vector.lengthSquared();

         double point1Percentage = 0.0;
         double point2Percentage = 1.0;
         double point3Percentage = point2Vector.dot(point3Vector) / oneToTwoSqaured;
         double point4Percentage = point2Vector.dot(point4Vector) / oneToTwoSqaured;

         intersectionToPack.setToNaN();

         // finding minimum
         if (point3Percentage > point1Percentage && point4Percentage > point1Percentage)
         {
            intersectionToPack.setFirstEndpoint(point3Percentage < point4Percentage ? point3 : point4);
         }
         else if (point3Percentage >= point1Percentage || point4Percentage >= point1Percentage)
         {
            intersectionToPack.setFirstEndpoint(point1);
         }

         // finding maximum
         if (point3Percentage < point2Percentage && point4Percentage < point2Percentage)
         {
            intersectionToPack.setSecondEndpoint(point3Percentage > point4Percentage ? point3 : point4);
         }
         else if (point3Percentage <= point2Percentage || point4Percentage <= point2Percentage)
         {
            intersectionToPack.setSecondEndpoint(point2);
         }

         if (intersectionToPack.firstEndpointContainsNaN() && !intersectionToPack.secondEndpointContainsNaN())
         {
            intersectionToPack.setFirstEndpoint(intersectionToPack.getSecondEndpoint());
         }
         else if (!intersectionToPack.firstEndpointContainsNaN() && intersectionToPack.secondEndpointContainsNaN())
         {
            intersectionToPack.setSecondEndpoint(intersectionToPack.getFirstEndpoint());
         }

         if (!intersectionToPack.containsNaN())
         {
            return;
         }
      }
      else if (allAreValid(point1, point2, point3))
      {
         intersectionToPack.set(point1, point2);
         if (intersectionToPack.isBetweenEndpoints(point3, Epsilons.ONE_TEN_MILLIONTH))
         {
            intersectionToPack.setFirstEndpoint(point3);
            intersectionToPack.setSecondEndpoint(point3);
            return;
         }
      }
      else if (allAreValid(point1, point3, point4))
      {
         intersectionToPack.set(point3, point3);
         if (intersectionToPack.isBetweenEndpoints(point1, Epsilons.ONE_TEN_MILLIONTH))
         {
            intersectionToPack.setFirstEndpoint(point1);
            intersectionToPack.setSecondEndpoint(point1);
            return;
         }
      }
      else if (allAreValid(point1, point3))
      {
         if (point1.epsilonEquals(point3, Epsilons.ONE_TEN_MILLIONTH))
         {
            intersectionToPack.setFirstEndpoint(point1);
            intersectionToPack.setSecondEndpoint(point3);
            return;
         }
      }

      noIntersection = true;
   }

   private boolean allAreValid(FramePoint3D point1, FramePoint3D point2)
   {
      return isValid(point1) && isValid(point2);
   }

   private boolean allAreValid(FramePoint3D point1, FramePoint3D point2, FramePoint3D point3)
   {
      return isValid(point1) && isValid(point2) && isValid(point3);
   }

   private boolean allAreValid(FramePoint3D point1, FramePoint3D point2, FramePoint3D point3, FramePoint3D point4)
   {
      return isValid(point1) && isValid(point2) && isValid(point3) && isValid(point4);
   }

   private boolean isValid(FramePoint3D point)
   {
      return !point.containsNaN();
   }

   private void intersectASinglePolygon(FrameConvexPolygon2D polygon, Pair<FramePoint3D, FramePoint3D> intersectionWithPolygon)
   {
      intersectionOfPlanes.changeFrame(polygon.getReferenceFrame());
      intersectionOfPlanes.set(planeIntersectionOnPolygonPlane);
      polygon.intersectionWith(planeIntersectionOnPolygonPlane, lineIntersectionOnPolygonPlane.getFirst(), lineIntersectionOnPolygonPlane.getSecond());

      if (lineIntersectionOnPolygonPlane.getFirst().containsNaN() && lineIntersectionOnPolygonPlane.getSecond().containsNaN())
      {
         noIntersection = true;
      }
      if (!lineIntersectionOnPolygonPlane.getFirst().containsNaN())
      {
         intersectionWithPolygon.getFirst().setIncludingFrame(lineIntersectionOnPolygonPlane.getFirst(), 0.0);
      }
      if (!lineIntersectionOnPolygonPlane.getSecond().containsNaN())
      {
         intersectionWithPolygon.getSecond().setIncludingFrame(lineIntersectionOnPolygonPlane.getSecond(), 0.0);
      }
   }

   /**
    * Gets the closest point to a filled polygon. If inside, packs that point.
    * If outside, the closest vertex or point along edge.
    */
   public static void getClosestPoint(Point2D point, ConvexPolygon2D polygon, Point2D closestPointToPack)
   {
      if (polygon.isPointInside(point))
      {
         closestPointToPack.set(point);
         return;
      }

      double closestDistance = Double.POSITIVE_INFINITY;
      for (int index = 0; index < polygon.getNumberOfVertices(); index++)
      {
         Point2DReadOnly pointOne = polygon.getVertex(index);
         Point2DReadOnly pointTwo = polygon.getNextVertex(index);

         boolean insideOne = EuclidGeometryTools.dotProduct(pointOne, pointTwo, pointOne, point) > 0.0;
         boolean insideTwo = EuclidGeometryTools.dotProduct(pointTwo, pointOne, pointTwo, point) > 0.0;

         if (insideOne && insideTwo)
         {
            double vx0 = point.getX() - pointOne.getX();
            double vy0 = point.getY() - pointOne.getY();

            double vx1 = pointTwo.getX() - pointOne.getX();
            double vy1 = pointTwo.getY() - pointOne.getY();

            double dot = vx0 * vx1 + vy0 * vy1;
            double lengthSquared = vx1 * vx1 + vy1 * vy1;

            double alpha = dot / lengthSquared;

            double x = pointOne.getX() + alpha * vx1;
            double y = pointOne.getY() + alpha * vy1;

            double distance = EuclidGeometryTools.distanceBetweenPoint2Ds(point.getX(), point.getY(), x, y);
            if (distance < closestDistance)
            {
               closestPointToPack.set(x, y);
               closestDistance = distance;
            }
         }
         else if (!insideOne)
         {
            double distance = point.distance(pointOne);
            if (distance < closestDistance)
            {
               closestPointToPack.set(pointOne);
               closestDistance = distance;
            }
         }
         else // (!insideTwo)
         {
            double distance = point.distance(pointTwo);
            if (distance < closestDistance)
            {
               closestPointToPack.set(pointTwo);
               closestDistance = distance;
            }
         }
      }
   }

   private static final ThreadLocal<Point3D> pointOnIntersectionThreadLocal = new ThreadLocal<Point3D>()
   {
      @Override
      public Point3D initialValue()
      {
         return new Point3D();
      }
   };
   private static final ThreadLocal<Vector3D> intersectionDirectionThreadLocal = new ThreadLocal<Vector3D>()
   {
      @Override
      public Vector3D initialValue()
      {
         return new Vector3D();
      }
   };

   public static void intersectTwoPlanes(FramePlane3d planeOne, FramePlane3d planeTwo, FrameLine3D intersectionToPack)
   {
      ReferenceFrame previousPlaneTwoReferenceFrame = planeTwo.getReferenceFrame();
      planeTwo.changeFrame(planeOne.getReferenceFrame());
      Point3D pointOnIntersection = pointOnIntersectionThreadLocal.get();
      Vector3DBasics intersectionDirection = intersectionDirectionThreadLocal.get();
      boolean success = EuclidGeometryTools.intersectionBetweenTwoPlane3Ds(planeOne.getPoint(), planeOne.getNormal(), planeTwo.getPoint(), planeTwo.getNormal(),
                                                                           pointOnIntersection, intersectionDirection);
      if (success)
      {
         intersectionToPack.setToZero(planeOne.getReferenceFrame());
         intersectionToPack.setPoint(pointOnIntersection);
         intersectionToPack.setDirection(intersectionDirection);
      }
      else
      {
         intersectionToPack.setToNaN();
      }

      planeTwo.changeFrame(previousPlaneTwoReferenceFrame);
   }
}
