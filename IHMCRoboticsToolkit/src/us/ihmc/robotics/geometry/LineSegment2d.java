package us.ihmc.robotics.geometry;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * A line segment must have two distinct endpoints by definition.
 *
 * @author Twan Koolen
 */
public class LineSegment2d implements GeometryObject<LineSegment2d>
{
   protected Point2D[] endpoints = new Point2D[2];
   
   public LineSegment2d()
   {
      endpoints[0] = new Point2D(Double.MIN_VALUE, Double.MIN_VALUE);
      endpoints[1] = new Point2D(Double.MAX_VALUE, Double.MAX_VALUE);
   }

   public LineSegment2d(double firstEndpointX, double firstEndpointY, double secondEndpointX, double secondEndpointY)
   {
      endpoints[0] = new Point2D(firstEndpointX, firstEndpointY);
      endpoints[1] = new Point2D(secondEndpointX, secondEndpointY);
      checkEndpointsDistinct(endpoints);
   }

   public LineSegment2d(Point2DReadOnly[] endpoints)
   {
      checkEndpointsDistinct(endpoints);
      this.endpoints[0] = new Point2D(endpoints[0]);
      this.endpoints[1] = new Point2D(endpoints[1]);
   }

   public LineSegment2d(Point2DReadOnly endpoint1, Point2DReadOnly endpoint2)
   {
      checkEndpointsDistinct(new Point2DReadOnly[] {endpoint1, endpoint2});
      endpoints[0] = new Point2D(endpoint1);
      endpoints[1] = new Point2D(endpoint2);
   }

   public LineSegment2d(LineSegment2d lineSegment2d)
   {
      endpoints = lineSegment2d.getEndpointsCopy();
   }

   public Point2D[] getEndpointsCopy()
   {
      return new Point2D[] {new Point2D(endpoints[0]), new Point2D(endpoints[1])};
   }

   public void getEndpoints(Point2DBasics endpoint0, Point2DBasics endpoint1)
   {
      endpoint0.set(endpoints[0]);
      endpoint1.set(endpoints[1]);
   }
   
   public Point2D getFirstEndpoint()
   {
      return endpoints[0];
   }
   
   public Point2D getSecondEndpoint()
   {
      return endpoints[1];
   }

   public Point2DReadOnly[] getEndpoints()
   {
      return endpoints;
   }

   public Point2D getFirstEndpointCopy()
   {
      return new Point2D(endpoints[0]);
   }

   public Point2D getSecondEndpointCopy()
   {
      return new Point2D(endpoints[1]);
   }

   public double getFirstEndpointX()
   {
      return endpoints[0].getX();
   }

   public double getFirstEndpointY()
   {
      return endpoints[0].getY();
   }

   public double getSecondEndpointX()
   {
      return endpoints[1].getX();
   }

   public double getSecondEndpointY()
   {
      return endpoints[1].getY();
   }

   public void set(Point2DReadOnly endpoint0, Point2DReadOnly endpoint1)
   {
      endpoints[0].set(endpoint0);
      endpoints[1].set(endpoint1);
      checkEndpointsDistinct(endpoints);
   }

   public void set(Point2DReadOnly endpoint0, Vector2DReadOnly fromPoint0ToPoint1)
   {
      endpoints[0].set(endpoint0);
      endpoints[1].set(endpoint0);
      endpoints[1].add(fromPoint0ToPoint1);
      checkEndpointsDistinct(endpoints);
   }

   public void set(double firstEndpointX, double firstEndpointY, double secondEndpointX, double secondEndpointY)
   {
      endpoints[0].set(firstEndpointX, firstEndpointY);
      endpoints[1].set(secondEndpointX, secondEndpointY);
      checkEndpointsDistinct(endpoints);
   }

   public void set(Point2DReadOnly[] endpoints)
   {
      if (endpoints.length != 2)
         throw new RuntimeException("Length of input array is not correct. Length = " + endpoints.length);
      this.endpoints[0].set(endpoints[0]);
      this.endpoints[1].set(endpoints[1]);
      checkEndpointsDistinct(endpoints);

   }

   @Override
   public void set(LineSegment2d lineSegment)
   {
      endpoints[0].set(lineSegment.endpoints[0]);
      endpoints[1].set(lineSegment.endpoints[1]);
   }

   public void flipDirection()
   {
      double xTemp = endpoints[0].getX();
      double yTemp = endpoints[0].getY();

      endpoints[0].set(endpoints[1]);
      endpoints[1].set(xTemp, yTemp);
   }

   public Point2D midpoint()
   {
      Point2D point2d = new Point2D();
      
      getMidpoint(point2d);

      return point2d;
   }
   
   public void getMidpoint(Point2DBasics midpointToPack)
   {
      midpointToPack.setX((endpoints[0].getX() + endpoints[1].getX()) / 2.0);
      midpointToPack.setY((endpoints[0].getY() + endpoints[1].getY()) / 2.0);
   }

   public double length()
   {
      return endpoints[0].distance(endpoints[1]);
   }

   public double dotProduct(LineSegment2d lineSegment2d)
   {
      double dotProduct = (endpoints[1].getX() - endpoints[0].getX()) * (lineSegment2d.endpoints[1].getX() - lineSegment2d.endpoints[0].getX())
                          + (endpoints[1].getY() - endpoints[0].getY()) * (lineSegment2d.endpoints[1].getY() - lineSegment2d.endpoints[0].getY());

      return dotProduct;
   }

   public boolean isBetweenEndpoints(Point2DReadOnly point2d, double epsilon)
   {
      return isBetweenEndpoints(point2d.getX(), point2d.getY(), epsilon);
   }
   
   private boolean isBetweenEndpoints(double x, double y, double epsilon)
   {
      double alpha = percentageAlongLineSegment(x, y);

      if (alpha < epsilon)
         return false;
      if (alpha > 1.0 - epsilon)
         return false;

      return true;
   }

   public boolean isPointOnLeftSideOfLineSegment(Point2DReadOnly point)
   {
      return GeometryTools.isPointOnSideOfLine(point, endpoints[0], endpoints[1], RobotSide.LEFT);
   }
   
   public boolean isPointOnRightSideOfLineSegment(Point2DReadOnly point)
   {
      return GeometryTools.isPointOnSideOfLine(point, endpoints[0], endpoints[1], RobotSide.RIGHT);
   }

   public LineSegment2d shiftToLeftCopy(double distanceToShift)
   {
      return shiftAndCopy(true, distanceToShift);
   }

   public LineSegment2d shiftToRightCopy(double distanceToShift)
   {
      return shiftAndCopy(false, distanceToShift);
   }

   private LineSegment2d shiftAndCopy(boolean shiftToLeft, double distanceToShift)
   {
      double vectorX = endpoints[1].getX() - endpoints[0].getX();
      double vectorY = endpoints[1].getY() - endpoints[0].getY();

      double vectorXPerpToRight = -vectorY;
      double vectorYPerpToRight = vectorX;

      if (!shiftToLeft)
      {
         vectorXPerpToRight = -vectorXPerpToRight;
         vectorYPerpToRight = -vectorYPerpToRight;
      }

      double vectorPerpToRightLength = Math.sqrt(vectorXPerpToRight * vectorXPerpToRight + vectorYPerpToRight * vectorYPerpToRight);
      vectorXPerpToRight = distanceToShift * vectorXPerpToRight / vectorPerpToRightLength;
      vectorYPerpToRight = distanceToShift * vectorYPerpToRight / vectorPerpToRightLength;

      Point2D newEndpoint0 = new Point2D(endpoints[0].getX() + vectorXPerpToRight, endpoints[0].getY() + vectorYPerpToRight);
      Point2D newEndpoint1 = new Point2D(endpoints[1].getX() + vectorXPerpToRight, endpoints[1].getY() + vectorYPerpToRight);

      LineSegment2d ret = new LineSegment2d(newEndpoint0, newEndpoint1);

      return ret;
   }

   public void shiftToLeft(double distanceToShift)
   {
      shift(true, distanceToShift);
   }

   public void shiftToRight(double distanceToShift)
   {
      shift(false, distanceToShift);
   }

   private void shift(boolean shiftToLeft, double distanceToShift)
   {
      double vectorX = endpoints[1].getX() - endpoints[0].getX();
      double vectorY = endpoints[1].getY() - endpoints[0].getY();

      double vectorXPerpToRight = -vectorY;
      double vectorYPerpToRight = vectorX;

      if (!shiftToLeft)
      {
         vectorXPerpToRight = -vectorXPerpToRight;
         vectorYPerpToRight = -vectorYPerpToRight;
      }

      double vectorPerpToRightLength = Math.sqrt(vectorXPerpToRight * vectorXPerpToRight + vectorYPerpToRight * vectorYPerpToRight);
      vectorXPerpToRight = distanceToShift * vectorXPerpToRight / vectorPerpToRightLength;
      vectorYPerpToRight = distanceToShift * vectorYPerpToRight / vectorPerpToRightLength;

      endpoints[0].setX(endpoints[0].getX() + vectorXPerpToRight);
      endpoints[0].setY(endpoints[0].getY() + vectorYPerpToRight);
      endpoints[1].setX(endpoints[1].getX() + vectorXPerpToRight);
      endpoints[1].setY(endpoints[1].getY() + vectorYPerpToRight);
   }

   /**
    * Computes a percentage along this line segment representing the location of the given point once projected onto this line segment.
    * The returned percentage is in ] -&infin;; &infin; [, {@code 0.0} representing {@code lineSegmentStart}, and {@code 1.0} representing {@code lineSegmentEnd}.
    * <p>
    * For example, if the returned percentage is {@code 0.5}, it means that the projection of the given point is located at the middle of the line segment.
    * The coordinates of the projection of the point can be computed from the {@code percentage} as follows:
    * <code>
    * Point2d projection = new Point2d(); </br>
    * projection.interpolate(lineSegmentStart, lineSegmentEnd, percentage); </br>
    * </code>
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if the length of the given line segment is too small, i.e. {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < Epsilons.ONE_TRILLIONTH}, this method fails and returns {@link Double#NaN}.
    * </ul>
    * </p>
    * 
    * @param pointX the x-coordinate of the query point.
    * @param pointY the y-coordinate of the query point.
    * @return the computed percentage along this line segment representing where the point projection is located.
    */
   public double percentageAlongLineSegment(Point2DReadOnly point2d)
   {
      return percentageAlongLineSegment(point2d.getX(), point2d.getY());
   }

   /**
    * Computes a percentage along this line segment representing the location of the given point once projected onto this line segment.
    * The returned percentage is in ] -&infin;; &infin; [, {@code 0.0} representing {@code lineSegmentStart}, and {@code 1.0} representing {@code lineSegmentEnd}.
    * <p>
    * For example, if the returned percentage is {@code 0.5}, it means that the projection of the given point is located at the middle of the line segment.
    * The coordinates of the projection of the point can be computed from the {@code percentage} as follows:
    * <code>
    * Point2d projection = new Point2d(); </br>
    * projection.interpolate(lineSegmentStart, lineSegmentEnd, percentage); </br>
    * </code>
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if the length of the given line segment is too small, i.e. {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < Epsilons.ONE_TRILLIONTH}, this method fails and returns {@link Double#NaN}.
    * </ul>
    * </p>
    * 
    * @param pointX the x-coordinate of the query point.
    * @param pointY the y-coordinate of the query point.
    * @return the computed percentage along this line segment representing where the point projection is located.
    */
   public double percentageAlongLineSegment(double x, double y)
   {
      return GeometryTools.getPercentageAlongLineSegment(x, y, endpoints[0].getX(), endpoints[0].getY(), endpoints[1].getX(), endpoints[1].getY());
   }

   public boolean isPointOnLineSegment(Point2DReadOnly point2d)
   {
      return isPointOnLineSegment(point2d.getX(), point2d.getY());
   }
   
   private boolean isPointOnLineSegment(double x, double y)
   {
      double vx0 = x - endpoints[0].getX();
      double vy0 = y - endpoints[0].getY();

      double vx1 = endpoints[1].getX() - endpoints[0].getX();
      double vy1 = endpoints[1].getY() - endpoints[0].getY();

      double cross = vx0 * vy1 - vy0 * vx1;
      boolean pointIsOnLine = Math.abs(cross) < 1e-7;

      return pointIsOnLine && isBetweenEndpoints(x, y, 0.0);
   }

   public Point2D intersectionWith(LineSegment2d secondLineSegment2d)
   {
      return GeometryTools.getIntersectionBetweenTwoLineSegments(endpoints[0], endpoints[1], secondLineSegment2d.endpoints[0], secondLineSegment2d.endpoints[1]);
   }
   
   public boolean intersectionWith(LineSegment2d secondLineSegment2d, Point2D intersectionToPack)
   {
      return GeometryTools.getIntersectionBetweenTwoLineSegments(endpoints[0], endpoints[1], secondLineSegment2d.endpoints[0], secondLineSegment2d.endpoints[1], intersectionToPack);
   }
   
   public Point2D intersectionWith(Line2d line2d)
   {
      return GeometryTools.getIntersectionBetweenLineAndLineSegment(line2d.point, line2d.normalizedVector, endpoints[0], endpoints[1]);
   }
   
   public Point2D[] intersectionWith(ConvexPolygon2d convexPolygon)
   {
      return ConvexPolygonTools.intersection(this, convexPolygon);
   }

   public double distance(Line2d line)
   {
      throw new RuntimeException("Not yet implemented");
   }

   public double distance(LineSegment2d lineSegment)
   {
      throw new RuntimeException("Not yet implemented");
   }

   public double distance(ConvexPolygon2d convexPolygon)
   {
      throw new RuntimeException("Not yet implemented");
   }

   private static void checkEndpointsDistinct(Point2DReadOnly[] endpoints)
   {
      if (areEndpointsTheSame(endpoints[0], endpoints[1]))
      {
         throw new RuntimeException("Line segment must have two distinct endpoints");
      }
   }

   public static boolean areEndpointsTheSame(Point2DReadOnly firstEndpoint, Point2DReadOnly secondEndpoint)
   {
      return areEndpointsTheSame(firstEndpoint.getX(), firstEndpoint.getY(), secondEndpoint.getX(), secondEndpoint.getY());
   }
   
   public static boolean areEndpointsTheSame(double firstEndpointX, double firstEndpointY, double secondEndpointX, double secondEndpointY)
   {
      return (firstEndpointX == secondEndpointX) && (firstEndpointY == secondEndpointY);
   }

   @Override
   public String toString()
   {
      return endpoints[0] + "-" + endpoints[1];
   }

   @Override
   public void applyTransform(Transform transform)
   {
      endpoints[0].applyTransform(transform);
      endpoints[1].applyTransform(transform);
   }

   public void applyTransformAndProjectToXYPlane(Transform transform)
   {
      endpoints[0].applyTransform(transform, false);
      endpoints[1].applyTransform(transform, false);
   }

   public LineSegment2d applyTransformCopy(Transform transform)
   {
      LineSegment2d copy = new LineSegment2d(this);
      copy.applyTransform(transform);
      return copy;
   }

   public LineSegment2d applyTransformAndProjectToXYPlaneCopy(Transform transform)
   {
      LineSegment2d copy = new LineSegment2d(this);
      copy.applyTransformAndProjectToXYPlane(transform);
      return copy;
   }

   public void pointBetweenEndPointsGivenParameter(double parameter, Point2DBasics pointToPack)
   {
      if ((parameter > 1.0) || (parameter < 0.0))
      {
         throw new RuntimeException("Parameter out of range. Parameter = " + parameter);
      }

      double x = endpoints[0].getX() + (endpoints[1].getX() - endpoints[0].getX()) * parameter;
      double y = endpoints[0].getY() + (endpoints[1].getY() - endpoints[0].getY()) * parameter;

      pointToPack.set(x, y);
   }
   
   public Point2D pointBetweenEndPointsGivenParameter(double parameter)
   {
      Point2D pointToReturn = new Point2D();
      
      pointBetweenEndPointsGivenParameter(parameter, pointToReturn);
      return pointToReturn;
   }

   /**
    * Compute the smallest distance from the point to this line segment.
    * If the projection of the given point on this line segment results in a point that is outside the line segment, the distance is computed between the given point and the closest line segment end point.
    */
   public double distance(Point2DReadOnly point)
   {
      double alpha = percentageAlongLineSegment(point);

      if (alpha <= 0.0)
      {
         return point.distance(endpoints[0]);
      }
      else if (alpha >= 1.0)
      {
         return point.distance(endpoints[1]);
      }
      else
      {
         // Here we know the projection of the point belongs to the line segment.
         // In this case computing the distance from the point to the line segment is the same as computing the distance from the point the equivalent line.
         return EuclidGeometryTools.distanceFromPoint2DToLine2D(point, endpoints[0], endpoints[1]);
      }
   }

   /**
    * Computes the orthogonal projection of a 2D point on this 2D line segment.
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if the length of this line segment is too small,
    *     i.e. {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < Epsilons.ONE_TRILLIONTH},
    *      this method fails and returns {@code false}.
    *    <li> the projection can not be outside the line segment.
    *     When the projection on the corresponding line is outside this line segment, the result is the closest of the two end points.
    * </ul>
    * </p>
    * 
    * @param point the point to compute the projection of. Not modified.
    * @return the projection of the point onto this line segment or {@code null} if the method failed.
    */
   public Point2D orthogonalProjectionCopy(Point2DReadOnly point)
   {
      return GeometryTools.getOrthogonalProjectionOnLineSegment(point, endpoints[0], endpoints[1]);
   }

   /**
    * Computes the orthogonal projection of a 2D point on this 2D line segment.
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if the length of this line segment is too small,
    *     i.e. {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < Epsilons.ONE_TRILLIONTH},
    *      this method fails and returns {@code false}.
    *    <li> the projection can not be outside the line segment.
    *     When the projection on the corresponding line is outside this line segment, the result is the closest of the two end points.
    * </ul>
    * </p>
    * 
    * @param point2d the point to project on this line segment. Modified.
    */
   public void orthogonalProjection(Point2DBasics point2d)
   {
      orthogonalProjection(point2d, point2d);
   }

   /**
    * Computes the orthogonal projection of a 2D point on this 2D line segment.
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if the length of this line segment is too small,
    *     i.e. {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < Epsilons.ONE_TRILLIONTH},
    *      this method fails and returns {@code false}.
    *    <li> the projection can not be outside the line segment.
    *     When the projection on the corresponding line is outside this line segment, the result is the closest of the two end points.
    * </ul>
    * </p>
    * 
    * @param point2d the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto this line segment is stored. Modified.
    * @return whether the method succeeded or not.
    */
   public boolean orthogonalProjection(Point2DReadOnly point2d, Point2DBasics projectedToPack)
   {
      return GeometryTools.getOrthogonalProjectionOnLineSegment(point2d, endpoints[0], endpoints[1], projectedToPack);
   }

   public Point2D getClosestPointOnLineSegmentCopy(Point2DReadOnly point2d)
   {
      return orthogonalProjectionCopy(point2d);
   }

   /**
    * This is the same calculation as for {@link #orthogonalProjection(Point2D, Point2D)}: </br>
    * Computes the orthogonal projection of a 2D point on this 2D line segment.
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if the length of this line segment is too small,
    *     i.e. {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < Epsilons.ONE_TRILLIONTH},
    *      this method fails and returns {@code false}.
    *    <li> the projection can not be outside the line segment.
    *     When the projection on the corresponding line is outside this line segment, the result is the closest of the two end points.
    * </ul>
    * </p>
    * 
    * @param point2d the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto this line segment is stored. Modified.
    * @return whether the method succeeded or not.
    */
   public boolean getClosestPointOnLineSegment(Point2DBasics closestPointToPack, Point2DReadOnly point2d)
   {
      return orthogonalProjection(point2d, closestPointToPack);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == null) return false;
      if (!(object instanceof LineSegment2d)) return false;
      if (object == this) return true;

      LineSegment2d otherSegment = (LineSegment2d) object;

      if (endpoints[0].equals(otherSegment.endpoints[0]) && endpoints[1].equals(otherSegment.endpoints[1]))
         return true;
      else if (endpoints[0].equals(otherSegment.endpoints[1]) && endpoints[1].equals(otherSegment.endpoints[0]))
         return true;

      return false;
   }

   public void getPerpendicularBisector(Vector2DBasics perpendicularBisectorToPack, double bisectorLengthDesired)
   {
      double x = endpoints[0].getX() - endpoints[1].getX();
      double y = endpoints[0].getY() - endpoints[1].getY();
      
      perpendicularBisectorToPack.set(-y, x);
      perpendicularBisectorToPack.normalize();
      perpendicularBisectorToPack.scale(bisectorLengthDesired);
   }

   @Override
   public void setToZero()
   {
      endpoints[0].setToZero();
      endpoints[1].setToZero();
   }

   @Override
   public void setToNaN()
   {
      endpoints[0].setToNaN();
      endpoints[1].setToNaN();      
   }

   @Override
   public boolean containsNaN()
   {
      if (endpoints[0].containsNaN()) return true;
      if (endpoints[1].containsNaN()) return true;
      
      return false;
   }

   @Override
   public boolean epsilonEquals(LineSegment2d other, double epsilon)
   {
      if (endpoints[0].epsilonEquals(other.endpoints[0], epsilon) && endpoints[1].epsilonEquals(other.endpoints[1], epsilon)) return true;
      if (endpoints[0].epsilonEquals(other.endpoints[1], epsilon) && endpoints[1].epsilonEquals(other.endpoints[0], epsilon)) return true;

      return false;
   }
}
