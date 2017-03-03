package us.ihmc.robotics.geometry.algorithms;

import org.apache.commons.math3.util.Pair;

import us.ihmc.commons.Epsilons;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.ConvexPolygon2dCalculator;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameLine;
import us.ihmc.robotics.geometry.FrameLine2d;
import us.ihmc.robotics.geometry.FrameLineSegment;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.geometry.Line2d;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.geometry.shapes.FramePlane3d;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

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
   private final FrameLine intersectionOfPlanes;
   private final FrameLine2d planeIntersectionOnPolygonPlane;
   private final Pair<FramePoint2d, FramePoint2d> lineIntersectionOnPolygonPlane;
   private final Pair<FramePoint, FramePoint> intersectionWithPolygonOne;
   private final Pair<FramePoint, FramePoint> intersectionWithPolygonTwo;
   private final Vector3D point2Vector;
   private final Vector3D point3Vector;
   private final Vector3D point4Vector;
   private boolean noIntersection;

   public FrameConvexPolygon2dIntersector()
   {
      planeOne = new FramePlane3d();
      planeTwo = new FramePlane3d();
      intersectionOfPlanes = new FrameLine();
      planeIntersectionOnPolygonPlane = new FrameLine2d();
      lineIntersectionOnPolygonPlane = new Pair<FramePoint2d, FramePoint2d>(new FramePoint2d(), new FramePoint2d());
      intersectionWithPolygonOne = new Pair<FramePoint, FramePoint>(new FramePoint(), new FramePoint());
      intersectionWithPolygonTwo = new Pair<FramePoint, FramePoint>(new FramePoint(), new FramePoint());
      point2Vector = new Vector3D();
      point3Vector = new Vector3D();
      point4Vector = new Vector3D();
      noIntersection = false;
   }

   public void intersect3d(FrameConvexPolygon2d polygonOne, FrameConvexPolygon2d polygonTwo, FrameLineSegment intersectionToPack)
   {
      noIntersection = false;

      polygonOne.getPlane3d(planeOne);
      polygonTwo.getPlane3d(planeTwo);
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

   private void findIntersectionAmongPoints(FramePoint point1, FramePoint point2, FramePoint point3, FramePoint point4, FrameLineSegment intersectionToPack)
   {
      if (allAreValid(point1, point2, point3, point4))
      {
         point2Vector.sub(point2.getPoint(), point1.getPoint());
         point3Vector.sub(point3.getPoint(), point1.getPoint());
         point4Vector.sub(point4.getPoint(), point1.getPoint());

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

   private boolean allAreValid(FramePoint point1, FramePoint point2)
   {
      return isValid(point1) && isValid(point2);
   }

   private boolean allAreValid(FramePoint point1, FramePoint point2, FramePoint point3)
   {
      return isValid(point1) && isValid(point2) && isValid(point3);
   }

   private boolean allAreValid(FramePoint point1, FramePoint point2, FramePoint point3, FramePoint point4)
   {
      return isValid(point1) && isValid(point2) && isValid(point3) && isValid(point4);
   }

   private boolean isValid(FramePoint point)
   {
      return !point.containsNaN();
   }

   private void intersectASinglePolygon(FrameConvexPolygon2d polygon, Pair<FramePoint, FramePoint> intersectionWithPolygon)
   {
      intersectionOfPlanes.changeFrame(polygon.getReferenceFrame());
      intersectionOfPlanes.projectOntoXYPlane(planeIntersectionOnPolygonPlane);
      polygon.intersectionWith(planeIntersectionOnPolygonPlane, lineIntersectionOnPolygonPlane);

      if (lineIntersectionOnPolygonPlane.getFirst().containsNaN() && lineIntersectionOnPolygonPlane.getSecond().containsNaN())
      {
         noIntersection = true;
      }
      if (!lineIntersectionOnPolygonPlane.getFirst().containsNaN())
      {
         intersectionWithPolygon.getFirst().setXYIncludingFrame(lineIntersectionOnPolygonPlane.getFirst());
      }
      if (!lineIntersectionOnPolygonPlane.getSecond().containsNaN())
      {
         intersectionWithPolygon.getSecond().setXYIncludingFrame(lineIntersectionOnPolygonPlane.getSecond());
      }
   }

   /**
    * Gets the closest point to a filled polygon. If inside, packs that point.
    * If outside, the closest vertex or point along edge.
    */
   public static void getClosestPoint(Point2D point, ConvexPolygon2d polygon, Point2D closestPointToPack)
   {
      if (ConvexPolygon2dCalculator.isPointInside(point, polygon))
      {
         closestPointToPack.set(point);
         return;
      }

      double closestDistance = Double.POSITIVE_INFINITY;
      for (int index = 0; index < polygon.getNumberOfVertices(); index++)
      {
         Point2DReadOnly pointOne = polygon.getVertex(index);
         Point2DReadOnly pointTwo = polygon.getNextVertex(index);

         boolean insideOne = GeometryTools.dotProduct(pointOne, pointTwo, pointOne, point) > 0.0;
         boolean insideTwo = GeometryTools.dotProduct(pointTwo, pointOne, pointTwo, point) > 0.0;

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

            double distance = GeometryTools.distanceBetweenPoints(point.getX(), point.getY(), x, y);
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

   private static final ThreadLocal<PoseReferenceFrame> tempIntersectPlaneReferenceFrame = new ThreadLocal<PoseReferenceFrame>()
   {
      @Override
      public PoseReferenceFrame initialValue()
      {
         return new PoseReferenceFrame("tempIntersectPlaneReferenceFrame", ReferenceFrame.getWorldFrame());
      }
   };
   private static final ThreadLocal<FrameLine> tempIntersectFrameLineOne = new ThreadLocal<FrameLine>()
   {
      @Override
      public FrameLine initialValue()
      {
         return new FrameLine();
      }
   };
   private static final ThreadLocal<FrameLine> tempIntersectFrameLineTwo = new ThreadLocal<FrameLine>()
   {
      @Override
      public FrameLine initialValue()
      {
         return new FrameLine();
      }
   };
   private static final ThreadLocal<Line2d> tempIntersectLine2dOne = new ThreadLocal<Line2d>()
   {
      @Override
      public Line2d initialValue()
      {
         return new Line2d();
      }
   };
   private static final ThreadLocal<Line2d> tempIntersectLine2dTwo = new ThreadLocal<Line2d>()
   {
      @Override
      public Line2d initialValue()
      {
         return new Line2d();
      }
   };
   private static final ThreadLocal<Quaternion> tempQuaternion = new ThreadLocal<Quaternion>()
   {
      @Override
      public Quaternion initialValue()
      {
         return new Quaternion();
      }
   };

   // TODO check out GeometryTools.getIntersectionBetweenTwoPlanes => no thread local and tested.
   public static void intersectTwoPlanes(FramePlane3d planeOne, FramePlane3d planeTwo, FrameLine intersectionToPack)
   {
      ReferenceFrame previousPlaneTwoReferenceFrame = planeTwo.getReferenceFrame();
      planeTwo.changeFrame(planeOne.getReferenceFrame());

      if (planeOne.isParallel(planeTwo, Epsilons.ONE_HUNDRED_MILLIONTH))
      {
         intersectionToPack.setToNaN();
      }
      else
      {
         intersectionToPack.setToZero(planeOne.getReferenceFrame());
         Vector3D intersectPlaneNormal = intersectionToPack.getNormalizedVector();
         intersectPlaneNormal.cross(planeOne.getNormal(), planeTwo.getNormal());
         intersectionToPack.setVectorWithoutChecks(intersectPlaneNormal);

         tempIntersectPlaneReferenceFrame.get().getOrientation(tempQuaternion.get());
         RotationTools.computeQuaternionFromYawAndZNormal(0.0, intersectionToPack.getNormalizedVector(),
                                                          tempQuaternion.get());
         tempIntersectPlaneReferenceFrame.get().setOrientationAndUpdate(tempQuaternion.get());
         tempIntersectPlaneReferenceFrame.get().setPositionWithoutChecksAndUpdate(planeOne.getPoint());

         tempIntersectFrameLineOne.get().setPointWithoutChecks(planeOne.getPoint());
         tempIntersectFrameLineTwo.get().setPointWithoutChecks(planeTwo.getPoint());

         Vector3D intersectVectorOne = tempIntersectFrameLineOne.get().getNormalizedVector();
         Vector3D intersectVectorTwo = tempIntersectFrameLineTwo.get().getNormalizedVector();
         intersectVectorOne.cross(planeOne.getNormal(), intersectionToPack.getNormalizedVector());
         intersectVectorTwo.cross(planeTwo.getNormal(), intersectionToPack.getNormalizedVector());
         tempIntersectFrameLineOne.get().setVectorWithoutChecks(intersectVectorOne);
         tempIntersectFrameLineTwo.get().setVectorWithoutChecks(intersectVectorTwo);

         tempIntersectFrameLineOne.get().changeFrame(tempIntersectPlaneReferenceFrame.get());
         tempIntersectFrameLineTwo.get().changeFrame(tempIntersectPlaneReferenceFrame.get());

         tempIntersectFrameLineOne.get().projectOntoXYPlane(tempIntersectLine2dOne.get());
         tempIntersectFrameLineTwo.get().projectOntoXYPlane(tempIntersectLine2dTwo.get());

         tempIntersectLine2dOne.get().intersectionWith(tempIntersectLine2dTwo.get(), intersectionToPack.getPoint());
      }

      planeTwo.changeFrame(previousPlaneTwoReferenceFrame);
   }
}
