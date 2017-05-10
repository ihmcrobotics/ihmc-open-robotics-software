package us.ihmc.robotics.geometry;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

/**
 * Represents a finite-length 2D line segment defined by its two 2D endpoints.
 */
public class LineSegment2d implements GeometryObject<LineSegment2d>
{
   /** The first endpoint defining this line segment. */
   private final Point2D firstEndpoint = new Point2D();
   /** The second endpoint defining this line segment. */
   private final Point2D secondEndpoint = new Point2D();

   /**
    * Default constructor that initializes both endpoints of this line segment to zero.
    */
   public LineSegment2d()
   {
   }

   /**
    * Creates a new line segment 2D and initializes it to {@code other}.
    * 
    * @param other the other line segment used to initialize this line segment. Not modified.
    */
   public LineSegment2d(LineSegment2d other)
   {
      set(other);
   }

   /**
    * Initializes this line segment to have the given endpoints.
    * 
    * @param firstEndpoint the first endpoint of this line segment. Not modified.
    * @param secondEndpoint the second endpoint of this line segment. Not modified.
    */
   public LineSegment2d(Point2DReadOnly firstEndpoint, Point2DReadOnly secondEndpoint)
   {
      set(firstEndpoint, secondEndpoint);
   }

   /**
    * Initializes this line segment to have the given endpoints.
    * 
    * @param firstEndpointX x-coordinate of the first endpoint of this line segment.
    * @param firstEndpointY y-coordinate of the first endpoint of this line segment.
    * @param secondEndpointX x-coordinate of the second endpoint of this line segment.
    * @param secondEndpointY y-coordinate of the second endpoint of this line segment.
    */
   public LineSegment2d(double firstEndpointX, double firstEndpointY, double secondEndpointX, double secondEndpointY)
   {
      set(firstEndpointX, firstEndpointY, secondEndpointX, secondEndpointY);
   }

   /**
    * Initializes this line segment to have the given endpoints.
    * 
    * @param endpoints a two-element array containing in order the first and second endpoints for
    *           this line segment. Not modified.
    * @throws IllegalArgumentException if the given array has a length different than 2.
    */
   public LineSegment2d(Point2DReadOnly[] endpoints)
   {
      set(endpoints);
   }

   /**
    * Changes the first endpoint of this line segment.
    * 
    * @param firstEndpointX x-coordinate of the new first endpoint.
    * @param firstEndpointY y-coordinate of the new first endpoint.
    */
   public void setFirstEndpoint(double firstEndpointX, double firstEndpointY)
   {
      firstEndpoint.set(firstEndpointX, firstEndpointY);
   }

   /**
    * Changes the first endpoint of this line segment.
    * 
    * @param firstEndpoint new endpoint of this line segment. Not modified
    */
   public void setFirstEndpoint(Point2DReadOnly firstEndpoint)
   {
      this.firstEndpoint.set(firstEndpoint);
   }

   /**
    * Changes the second endpoint of this line segment.
    * 
    * @param secondEndpointX x-coordinate of the new second endpoint.
    * @param secondEndpointY y-coordinate of the new second endpoint.
    */
   public void setSecondEndpoint(double secondEndpointX, double secondEndpointY)
   {
      secondEndpoint.set(secondEndpointX, secondEndpointY);
   }

   /**
    * Changes the second endpoint of this line segment.
    * 
    * @param secondEndpoint new second endpoint of this line segment. Not modified.
    */
   public void setSecondEndpoint(Point2DReadOnly secondEndpoint)
   {
      this.secondEndpoint.set(secondEndpoint);
   }

   /**
    * Redefines this line segments with new endpoints.
    * 
    * @param firstEndpointX x-coordinate of the new first endpoint.
    * @param firstEndpointY y-coordinate of the new first endpoint.
    * @param secondEndpointX x-coordinate of the new second endpoint.
    * @param secondEndpointY y-coordinate of the new second endpoint.
    */
   public void set(double firstEndpointX, double firstEndpointY, double secondEndpointX, double secondEndpointY)
   {
      firstEndpoint.set(firstEndpointX, firstEndpointY);
      secondEndpoint.set(secondEndpointX, secondEndpointY);
   }

   /**
    * Redefines this line segment with new endpoints.
    * 
    * @param firstEndpoint new endpoint of this line segment. Not modified
    * @param secondEndpoint new second endpoint of this line segment. Not modified.
    */
   public void set(Point2DReadOnly firstEndpoint, Point2DReadOnly secondEndpoint)
   {
      setFirstEndpoint(firstEndpoint);
      setSecondEndpoint(secondEndpoint);
   }

   /**
    * Redefines this line segment with a new first endpoint and a vector going from the first to the
    * second endpoint.
    * 
    * @param firstEndpoint new first endpoint. Not modified.
    * @param fromFirstToSecondEndpoint vector going from the first to the second endpoint. Not
    *           modified.
    */
   public void set(Point2DReadOnly firstEndpoint, Vector2DReadOnly fromFirstToSecondEndpoint)
   {
      this.firstEndpoint.set(firstEndpoint);
      secondEndpoint.add(firstEndpoint, fromFirstToSecondEndpoint);
   }

   /**
    * Redefines this line segment with new endpoints.
    * 
    * @param endpoints a two-element array containing in order the first and second endpoints for
    *           this line segment. Not modified.
    * @throws IllegalArgumentException if the given array has a length different than 2.
    */
   public void set(Point2DReadOnly[] endpoints)
   {
      if (endpoints.length != 2)
         throw new RuntimeException("Length of input array is not correct. Length = " + endpoints.length + ", expected an array of two elements");
      set(endpoints[0], endpoints[1]);
   }

   /**
    * Sets this line segment to be same as the given line segment.
    * 
    * @param other the other line segment to copy. Not modified.
    */
   @Override
   public void set(LineSegment2d other)
   {
      set(other.firstEndpoint, other.secondEndpoint);
   }

   /**
    * Sets both endpoints of this line segment to zero.
    */
   @Override
   public void setToZero()
   {
      firstEndpoint.setToZero();
      secondEndpoint.setToZero();
   }

   /**
    * Sets both endpoints of this line segment to {@link Double#NaN}. After calling this method,
    * this line segment becomes invalid. A new pair of valid endpoints will have to be set so this
    * line segment is again usable.
    */
   @Override
   public void setToNaN()
   {
      firstEndpoint.setToNaN();
      secondEndpoint.setToNaN();
   }

   /**
    * Tests if this line segment contains {@link Double#NaN}.
    * 
    * @return {@code true} if {@link #firstEndpoint} and/or {@link #secondEndpoint} contains
    *         {@link Double#NaN}, {@code false} otherwise.
    */
   @Override
   public boolean containsNaN()
   {
      return firstEndpoint.containsNaN() || secondEndpoint.containsNaN();
   }

   /**
    * Test if the first endpoint of this line segment contains {@link Double#NaN}.
    * 
    * @return {@code true} if {@link #firstEndpoint} contains {@link Double#NaN}, {@code false}
    *         otherwise.
    */
   public boolean firstEndpointContainsNaN()
   {
      return firstEndpoint.containsNaN();
   }

   /**
    * Test if the second endpoint of this line segment contains {@link Double#NaN}.
    * 
    * @return {@code true} if {@link #secondEndpoint} contains {@link Double#NaN}, {@code false}
    *         otherwise.
    */
   public boolean secondEndpointContainsNaN()
   {
      return secondEndpoint.containsNaN();
   }

   /**
    * Computes the length of this line segment.
    * 
    * @return the length of this line segment.
    */
   public double length()
   {
      return firstEndpoint.distance(secondEndpoint);
   }

   public void flipDirection()
   {
      double x = firstEndpoint.getX();
      double y = firstEndpoint.getY();

      firstEndpoint.set(secondEndpoint);
      secondEndpoint.set(x, y);
   }

   public Point2D midpoint()
   {
      Point2D midpoint = new Point2D();
      getMidpoint(midpoint);
      return midpoint;
   }

   public void getMidpoint(Point2DBasics midpointToPack)
   {
      midpointToPack.interpolate(firstEndpoint, secondEndpoint, 0.5);
   }

   public double dotProduct(LineSegment2d lineSegment2d)
   {
      return EuclidGeometryTools.dotProduct(firstEndpoint, secondEndpoint, lineSegment2d.firstEndpoint, lineSegment2d.secondEndpoint);
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
      return EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D(point, firstEndpoint, secondEndpoint);
   }

   public boolean isPointOnRightSideOfLineSegment(Point2DReadOnly point)
   {
      return EuclidGeometryTools.isPoint2DOnRightSideOfLine2D(point, firstEndpoint, secondEndpoint);
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
      double vectorX = secondEndpoint.getX() - firstEndpoint.getX();
      double vectorY = secondEndpoint.getY() - firstEndpoint.getY();

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

      Point2D newEndpoint0 = new Point2D(firstEndpoint.getX() + vectorXPerpToRight, firstEndpoint.getY() + vectorYPerpToRight);
      Point2D newEndpoint1 = new Point2D(secondEndpoint.getX() + vectorXPerpToRight, secondEndpoint.getY() + vectorYPerpToRight);

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
      double vectorX = secondEndpoint.getX() - firstEndpoint.getX();
      double vectorY = secondEndpoint.getY() - firstEndpoint.getY();

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

      firstEndpoint.setX(firstEndpoint.getX() + vectorXPerpToRight);
      firstEndpoint.setY(firstEndpoint.getY() + vectorYPerpToRight);
      secondEndpoint.setX(secondEndpoint.getX() + vectorXPerpToRight);
      secondEndpoint.setY(secondEndpoint.getY() + vectorYPerpToRight);
   }

   /**
    * Computes a percentage along this line segment representing the location of the given point
    * once projected onto this line segment. The returned percentage is in ] -&infin;; &infin; [,
    * {@code 0.0} representing {@code lineSegmentStart}, and {@code 1.0} representing
    * {@code lineSegmentEnd}.
    * <p>
    * For example, if the returned percentage is {@code 0.5}, it means that the projection of the
    * given point is located at the middle of the line segment. The coordinates of the projection of
    * the point can be computed from the {@code percentage} as follows: <code>
    * Point2d projection = new Point2d(); </br>
    * projection.interpolate(lineSegmentStart, lineSegmentEnd, percentage); </br>
    * </code>
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the given line segment is too small, i.e.
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < Epsilons.ONE_TRILLIONTH}, this
    * method fails and returns {@link Double#NaN}.
    * </ul>
    * </p>
    * 
    * @param pointX the x-coordinate of the query point.
    * @param pointY the y-coordinate of the query point.
    * @return the computed percentage along this line segment representing where the point
    *         projection is located.
    */
   public double percentageAlongLineSegment(Point2DReadOnly point2d)
   {
      return percentageAlongLineSegment(point2d.getX(), point2d.getY());
   }

   /**
    * Computes a percentage along this line segment representing the location of the given point
    * once projected onto this line segment. The returned percentage is in ] -&infin;; &infin; [,
    * {@code 0.0} representing {@code lineSegmentStart}, and {@code 1.0} representing
    * {@code lineSegmentEnd}.
    * <p>
    * For example, if the returned percentage is {@code 0.5}, it means that the projection of the
    * given point is located at the middle of the line segment. The coordinates of the projection of
    * the point can be computed from the {@code percentage} as follows: <code>
    * Point2d projection = new Point2d(); </br>
    * projection.interpolate(lineSegmentStart, lineSegmentEnd, percentage); </br>
    * </code>
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the given line segment is too small, i.e.
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < Epsilons.ONE_TRILLIONTH}, this
    * method fails and returns {@link Double#NaN}.
    * </ul>
    * </p>
    * 
    * @param pointX the x-coordinate of the query point.
    * @param pointY the y-coordinate of the query point.
    * @return the computed percentage along this line segment representing where the point
    *         projection is located.
    */
   public double percentageAlongLineSegment(double x, double y)
   {
      return EuclidGeometryTools.percentageAlongLineSegment2D(x, y, firstEndpoint.getX(), firstEndpoint.getY(), secondEndpoint.getX(), secondEndpoint.getY());
   }

   public boolean isPointOnLineSegment(Point2DReadOnly point2d)
   {
      return isPointOnLineSegment(point2d.getX(), point2d.getY());
   }

   private boolean isPointOnLineSegment(double x, double y)
   {
      double vx0 = x - firstEndpoint.getX();
      double vy0 = y - firstEndpoint.getY();

      double vx1 = secondEndpoint.getX() - firstEndpoint.getX();
      double vy1 = secondEndpoint.getY() - firstEndpoint.getY();

      double cross = vx0 * vy1 - vy0 * vx1;
      boolean pointIsOnLine = Math.abs(cross) < 1e-7;

      return pointIsOnLine && isBetweenEndpoints(x, y, 0.0);
   }

   public Point2D intersectionWith(LineSegment2d secondLineSegment2d)
   {
      return EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(firstEndpoint, secondEndpoint, secondLineSegment2d.firstEndpoint,
                                                                      secondLineSegment2d.secondEndpoint);
   }

   public boolean intersectionWith(LineSegment2d secondLineSegment2d, Point2D intersectionToPack)
   {
      return EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(firstEndpoint, secondEndpoint, secondLineSegment2d.firstEndpoint,
                                                                      secondLineSegment2d.secondEndpoint, intersectionToPack);
   }

   public Point2D intersectionWith(Line2D line2d)
   {
      return EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(line2d.getPoint(), line2d.getDirection(), firstEndpoint, secondEndpoint);
   }

   public Point2D[] intersectionWith(ConvexPolygon2d convexPolygon)
   {
      return convexPolygon.intersectionWith(this);
   }

   public double distance(Line2D line)
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

   @Override
   public String toString()
   {
      return firstEndpoint + "-" + secondEndpoint;
   }

   public Point2D[] getEndpointsCopy()
   {
      return new Point2D[] {new Point2D(firstEndpoint), new Point2D(secondEndpoint)};
   }

   public void getEndpoints(Point2DBasics firstEndpointToPack, Point2DBasics secondEndpointToPack)
   {
      firstEndpointToPack.set(firstEndpoint);
      secondEndpointToPack.set(secondEndpoint);
   }

   public Point2DReadOnly getFirstEndpoint()
   {
      return firstEndpoint;
   }

   public Point2DReadOnly getSecondEndpoint()
   {
      return secondEndpoint;
   }

   public Point2DReadOnly getEndpoint(int index)
   {
      switch (index)
      {
      case 0:
         return firstEndpoint;
      case 1:
         return secondEndpoint;
      default:
         throw new IndexOutOfBoundsException(Integer.toString(index));
      }
   }

   public Point2D getFirstEndpointCopy()
   {
      return new Point2D(firstEndpoint);
   }

   public Point2D getSecondEndpointCopy()
   {
      return new Point2D(secondEndpoint);
   }

   public double getFirstEndpointX()
   {
      return firstEndpoint.getX();
   }

   public double getFirstEndpointY()
   {
      return firstEndpoint.getY();
   }

   public double getSecondEndpointX()
   {
      return secondEndpoint.getX();
   }

   public double getSecondEndpointY()
   {
      return secondEndpoint.getY();
   }

   @Override
   public void applyTransform(Transform transform)
   {
      firstEndpoint.applyTransform(transform);
      secondEndpoint.applyTransform(transform);
   }

   public void applyTransformAndProjectToXYPlane(Transform transform)
   {
      firstEndpoint.applyTransform(transform, false);
      secondEndpoint.applyTransform(transform, false);
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

      double x = firstEndpoint.getX() + (secondEndpoint.getX() - firstEndpoint.getX()) * parameter;
      double y = firstEndpoint.getY() + (secondEndpoint.getY() - firstEndpoint.getY()) * parameter;

      pointToPack.set(x, y);
   }

   public Point2D pointBetweenEndPointsGivenParameter(double parameter)
   {
      Point2D pointToReturn = new Point2D();

      pointBetweenEndPointsGivenParameter(parameter, pointToReturn);
      return pointToReturn;
   }

   /**
    * Compute the smallest distance from the point to this line segment. If the projection of the
    * given point on this line segment results in a point that is outside the line segment, the
    * distance is computed between the given point and the closest line segment end point.
    */
   public double distance(Point2DReadOnly point)
   {
      double alpha = percentageAlongLineSegment(point);

      if (alpha <= 0.0)
      {
         return point.distance(firstEndpoint);
      }
      else if (alpha >= 1.0)
      {
         return point.distance(secondEndpoint);
      }
      else
      {
         // Here we know the projection of the point belongs to the line segment.
         // In this case computing the distance from the point to the line segment is the same as computing the distance from the point the equivalent line.
         return EuclidGeometryTools.distanceFromPoint2DToLine2D(point, firstEndpoint, secondEndpoint);
      }
   }

   /**
    * Computes the orthogonal projection of a 2D point on this 2D line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of this line segment is too small, i.e.
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < Epsilons.ONE_TRILLIONTH}, this
    * method fails and returns {@code false}.
    * <li>the projection can not be outside the line segment. When the projection on the
    * corresponding line is outside this line segment, the result is the closest of the two end
    * points.
    * </ul>
    * </p>
    * 
    * @param point the point to compute the projection of. Not modified.
    * @return the projection of the point onto this line segment or {@code null} if the method
    *         failed.
    */
   public Point2D orthogonalProjectionCopy(Point2DReadOnly point)
   {
      return EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(point, firstEndpoint, secondEndpoint);
   }

   /**
    * Computes the orthogonal projection of a 2D point on this 2D line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of this line segment is too small, i.e.
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < Epsilons.ONE_TRILLIONTH}, this
    * method fails and returns {@code false}.
    * <li>the projection can not be outside the line segment. When the projection on the
    * corresponding line is outside this line segment, the result is the closest of the two end
    * points.
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
    * <li>if the length of this line segment is too small, i.e.
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < Epsilons.ONE_TRILLIONTH}, this
    * method fails and returns {@code false}.
    * <li>the projection can not be outside the line segment. When the projection on the
    * corresponding line is outside this line segment, the result is the closest of the two end
    * points.
    * </ul>
    * </p>
    * 
    * @param point2d the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto this line segment is
    *           stored. Modified.
    * @return whether the method succeeded or not.
    */
   public boolean orthogonalProjection(Point2DReadOnly point2d, Point2DBasics projectedToPack)
   {
      return EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(point2d, firstEndpoint, secondEndpoint, projectedToPack);
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
    * <li>if the length of this line segment is too small, i.e.
    * {@code lineSegmentStart.distanceSquared(lineSegmentEnd) < Epsilons.ONE_TRILLIONTH}, this
    * method fails and returns {@code false}.
    * <li>the projection can not be outside the line segment. When the projection on the
    * corresponding line is outside this line segment, the result is the closest of the two end
    * points.
    * </ul>
    * </p>
    * 
    * @param point2d the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto this line segment is
    *           stored. Modified.
    * @return whether the method succeeded or not.
    */
   public boolean getClosestPointOnLineSegment(Point2DBasics closestPointToPack, Point2DReadOnly point2d)
   {
      return orthogonalProjection(point2d, closestPointToPack);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == null)
         return false;
      if (!(object instanceof LineSegment2d))
         return false;
      if (object == this)
         return true;

      LineSegment2d otherSegment = (LineSegment2d) object;

      if (firstEndpoint.equals(otherSegment.firstEndpoint) && secondEndpoint.equals(otherSegment.secondEndpoint))
         return true;
      else if (firstEndpoint.equals(otherSegment.secondEndpoint) && secondEndpoint.equals(otherSegment.firstEndpoint))
         return true;

      return false;
   }

   public void getPerpendicularBisector(Vector2DBasics perpendicularBisectorToPack, double bisectorLengthDesired)
   {
      double x = firstEndpoint.getX() - secondEndpoint.getX();
      double y = firstEndpoint.getY() - secondEndpoint.getY();

      perpendicularBisectorToPack.set(-y, x);
      perpendicularBisectorToPack.normalize();
      perpendicularBisectorToPack.scale(bisectorLengthDesired);
   }

   @Override
   public boolean epsilonEquals(LineSegment2d other, double epsilon)
   {
      if (firstEndpoint.epsilonEquals(other.firstEndpoint, epsilon) && secondEndpoint.epsilonEquals(other.secondEndpoint, epsilon))
         return true;
      if (firstEndpoint.epsilonEquals(other.secondEndpoint, epsilon) && secondEndpoint.epsilonEquals(other.firstEndpoint, epsilon))
         return true;

      return false;
   }
}
