package us.ihmc.robotics.geometry;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;

import us.ihmc.robotics.geometry.transformables.TransformablePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * A line segment must have two distinct endpoints by definition.
 *
 * @author Twan Koolen
 */
public class LineSegment2d implements Geometry2d<LineSegment2d>
{
   protected TransformablePoint2d[] endpoints = new TransformablePoint2d[2];
   
   public LineSegment2d()
   {
      endpoints[0] = new TransformablePoint2d(Double.MIN_VALUE, Double.MIN_VALUE);
      endpoints[1] = new TransformablePoint2d(Double.MAX_VALUE, Double.MAX_VALUE);
   }

   public LineSegment2d(double firstEndpointX, double firstEndpointY, double secondEndpointX, double secondEndpointY)
   {
      endpoints[0] = new TransformablePoint2d(firstEndpointX, firstEndpointY);
      endpoints[1] = new TransformablePoint2d(secondEndpointX, secondEndpointY);
      checkEndpointsDistinct(endpoints);
   }

   public LineSegment2d(Point2d[] endpoints)
   {
      checkEndpointsDistinct(endpoints);
      this.endpoints[0] = new TransformablePoint2d(endpoints[0]);
      this.endpoints[1] = new TransformablePoint2d(endpoints[1]);
   }

   public LineSegment2d(Point2d endpoint1, Point2d endpoint2)
   {
      checkEndpointsDistinct(new Point2d[] {endpoint1, endpoint2});
      endpoints[0] = new TransformablePoint2d(endpoint1);
      endpoints[1] = new TransformablePoint2d(endpoint2);
   }

   public LineSegment2d(LineSegment2d lineSegment2d)
   {
      endpoints = lineSegment2d.getEndpointsCopy();
   }

   public TransformablePoint2d[] getEndpointsCopy()
   {
      return new TransformablePoint2d[] {new TransformablePoint2d(endpoints[0]), new TransformablePoint2d(endpoints[1])};
   }

   public void getEndpoints(Point2d endpoint0, Point2d endpoint1)
   {
      endpoint0.set(endpoints[0]);
      endpoint1.set(endpoints[1]);
   }
   
   public Point2d getFirstEndpoint()
   {
      return endpoints[0];
   }
   
   public Point2d getSecondEndpoint()
   {
      return endpoints[1];
   }

   public Point2d[] getEndpoints()
   {
      return endpoints;
   }

   public Point2d getFirstEndpointCopy()
   {
      return new Point2d(endpoints[0]);
   }

   public Point2d getSecondEndpointCopy()
   {
      return new Point2d(endpoints[1]);
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

   public void set(Point2d endpoint0, Point2d endpoint1)
   {
      endpoints[0].set(endpoint0);
      endpoints[1].set(endpoint1);
      checkEndpointsDistinct(endpoints);
   }

   public void set(Point2d endpoint0, Vector2d fromPoint0ToPoint1)
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

   public void set(Point2d[] endpoints)
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

   public Point2d midpoint()
   {
      Point2d point2d = new Point2d();
      
      getMidpoint(point2d);

      return point2d;
   }
   
   public void getMidpoint(Point2d midpointToPack)
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

   public boolean isBetweenEndpoints(Point2d point2d, double epsilon)
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

   public boolean isPointOnLeftSideOfLineSegment(Point2d point)
   {
      return GeometryTools.isPointOnSideOfLine(point, endpoints[0], endpoints[1], RobotSide.LEFT);
   }
   
   public boolean isPointOnRightSideOfLineSegment(Point2d point)
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

      Point2d newEndpoint0 = new Point2d(endpoints[0].getX() + vectorXPerpToRight, endpoints[0].getY() + vectorYPerpToRight);
      Point2d newEndpoint1 = new Point2d(endpoints[1].getX() + vectorXPerpToRight, endpoints[1].getY() + vectorYPerpToRight);

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
   public double percentageAlongLineSegment(Point2d point2d)
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

   public boolean isPointOnLineSegment(Point2d point2d)
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

   @Override
   public Point2d intersectionWith(LineSegment2d secondLineSegment2d)
   {
      return GeometryTools.getIntersectionBetweenTwoLineSegments(endpoints[0], endpoints[1], secondLineSegment2d.endpoints[0], secondLineSegment2d.endpoints[1]);
   }
   
   public boolean intersectionWith(LineSegment2d secondLineSegment2d, Point2d intersectionToPack)
   {
      return GeometryTools.getIntersectionBetweenTwoLineSegments(endpoints[0], endpoints[1], secondLineSegment2d.endpoints[0], secondLineSegment2d.endpoints[1], intersectionToPack);
   }
   
   @Override
   public Point2d intersectionWith(Line2d line2d)
   {
      return GeometryTools.getIntersectionBetweenLineAndLineSegment(line2d.point, line2d.normalizedVector, endpoints[0], endpoints[1]);
   }
   
   @Override
   public Point2d[] intersectionWith(ConvexPolygon2d convexPolygon)
   {
      return ConvexPolygonTools.intersection(this, convexPolygon);
   }

   @Override
   public double distance(Line2d line)
   {
      throw new RuntimeException("Not yet implemented");
   }

   @Override
   public double distance(LineSegment2d lineSegment)
   {
      throw new RuntimeException("Not yet implemented");
   }

   @Override
   public double distance(ConvexPolygon2d convexPolygon)
   {
      throw new RuntimeException("Not yet implemented");
   }

   private static void checkEndpointsDistinct(Point2d[] endpoints)
   {
      if (areEndpointsTheSame(endpoints[0], endpoints[1]))
      {
         throw new RuntimeException("Line segment must have two distinct endpoints");
      }
   }

   public static boolean areEndpointsTheSame(Point2d firstEndpoint, Point2d secondEndpoint)
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
   public void applyTransform(RigidBodyTransform transform)
   {
      checkIsTransformationInPlane(transform);
      applyTransformAndProjectToXYPlane(transform);
   }

   private Point3d tempTransformedPoint;

   @Override
   public void applyTransformAndProjectToXYPlane(RigidBodyTransform transform)
   {
      if (tempTransformedPoint == null)
         tempTransformedPoint = new Point3d();

      for (int i = 0; i < endpoints.length; i++)
      {
         tempTransformedPoint.set(endpoints[0].getX(), endpoints[0].getY(), 0.0);
         transform.transform(tempTransformedPoint);
         endpoints[0].set(tempTransformedPoint.getX(), tempTransformedPoint.getY());
      }
   }

   @Override
   public LineSegment2d applyTransformCopy(RigidBodyTransform transform)
   {
      LineSegment2d copy = new LineSegment2d(this);
      copy.applyTransform(transform);
      return copy;
   }

   @Override
   public LineSegment2d applyTransformAndProjectToXYPlaneCopy(RigidBodyTransform transform)
   {
      LineSegment2d copy = new LineSegment2d(this);
      copy.applyTransformAndProjectToXYPlane(transform);
      return copy;
   }

   public void pointBetweenEndPointsGivenParameter(double parameter, Point2d pointToPack)
   {
      if ((parameter > 1.0) || (parameter < 0.0))
      {
         throw new RuntimeException("Parameter out of range. Parameter = " + parameter);
      }

      double x = endpoints[0].getX() + (endpoints[1].getX() - endpoints[0].getX()) * parameter;
      double y = endpoints[0].getY() + (endpoints[1].getY() - endpoints[0].getY()) * parameter;

      pointToPack.set(x, y);
   }
   
   public Point2d pointBetweenEndPointsGivenParameter(double parameter)
   {
      Point2d pointToReturn = new Point2d();
      
      pointBetweenEndPointsGivenParameter(parameter, pointToReturn);
      return pointToReturn;
   }

   private final Matrix3d tempRotation = new Matrix3d();

   private boolean isTransformationInPlane(RigidBodyTransform transform)
   {
      // arguably not a sufficient condition. ReferenceFrame2d needed!
      transform.getRotation(tempRotation);

      return ReferenceFrame.isRotationInPlane(tempRotation);
   }

   private void checkIsTransformationInPlane(RigidBodyTransform transform)
   {
      if (!isTransformationInPlane(transform))
      {
         throw new RuntimeException("Cannot transform FrameLineSegment2d to a plane with a different surface normal");
      }
   }

   /**
    * Compute the smallest distance from the point to this line segment.
    * If the projection of the given point on this line segment results in a point that is outside the line segment, the distance is computed between the given point and the closest line segment end point.
    */
   @Override
   public double distance(Point2d point)
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
         return GeometryTools.distanceFromPointToLine(point, endpoints[0], endpoints[1]);
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
   @Override
   public Point2d orthogonalProjectionCopy(Point2d point)
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
   @Override
   public void orthogonalProjection(Point2d point2d)
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
   public boolean orthogonalProjection(Point2d point2d, Point2d projectedToPack)
   {
      return GeometryTools.getOrthogonalProjectionOnLineSegment(point2d, endpoints[0], endpoints[1], projectedToPack);
   }

   public Point2d getClosestPointOnLineSegmentCopy(Point2d point2d)
   {
      return orthogonalProjectionCopy(point2d);
   }

   /**
    * This is the same calculation as for {@link #orthogonalProjection(Point2d, Point2d)}: </br>
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
   public boolean getClosestPointOnLineSegment(Point2d closestPointToPack, Point2d point2d)
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

   public void getPerpendicularBisector(Vector2d perpendicularBisectorToPack, double bisectorLengthDesired)
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
