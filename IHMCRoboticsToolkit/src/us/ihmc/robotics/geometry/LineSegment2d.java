package us.ihmc.robotics.geometry;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
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

   /**
    * Computes the squared value of the length of this line segment.
    * 
    * @return the length squared of this line segment.
    */
   public double lengthSquared()
   {
      return firstEndpoint.distanceSquared(secondEndpoint);
   }

   /**
    * Returns the square of the minimum distance between a point and this given line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code this.lengthSquared() < Epsilons.ONE_TRILLIONTH}, this method returns the
    * distance between {@code firstEndpoint} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param point 3D point to compute the distance from this line segment. Not modified.
    * @return the minimum distance between the 2D point and this 2D line segment.
    */
   public double distanceSquared(Point2DReadOnly point)
   {
      return EuclidGeometryTools.distanceSquaredFromPoint2DToLineSegment2D(point.getX(), point.getY(), firstEndpoint, secondEndpoint);
   }

   /**
    * Returns the minimum distance between a point and this given line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code this.lengthSquared() < Epsilons.ONE_TRILLIONTH}, this method returns the
    * distance between {@code firstEndpoint} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param point 3D point to compute the distance from this line segment. Not modified.
    * @return the minimum distance between the 2D point and this 2D line segment.
    */
   public double distance(Point2DReadOnly point)
   {
      return EuclidGeometryTools.distanceFromPoint2DToLineSegment2D(point, firstEndpoint, secondEndpoint);
   }

   /**
    * Computes the orthogonal projection of a 2D point on this 2D line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of this line segment is too small, i.e.
    * {@code this.lengthSquared() < Epsilons.ONE_TRILLIONTH}, this method returns
    * {@code firstEndpoint}.
    * <li>the projection can not be outside the line segment. When the projection on the
    * corresponding line is outside the line segment, the result is the closest of the two
    * endpoints.
    * </ul>
    * </p>
    * 
    * @param pointToProject the point to compute the projection of. Not modified.
    * @return the projection of the point onto this line segment or {@code null} if the method
    *         failed.
    */
   public Point2D orthogonalProjectionCopy(Point2DReadOnly pointToProject)
   {
      return EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(pointToProject, firstEndpoint, secondEndpoint);
   }

   /**
    * Computes the orthogonal projection of a 2D point on this 2D line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of this line segment is too small, i.e.
    * {@code this.lengthSquared() < Epsilons.ONE_TRILLIONTH}, this method returns
    * {@code firstEndpoint}.
    * <li>the projection can not be outside the line segment. When the projection on the
    * corresponding line is outside the line segment, the result is the closest of the two
    * endpoints.
    * </ul>
    * </p>
    * 
    * @param pointToProject the point to project on this line segment. Modified.
    * @return whether the method succeeded or not.
    */
   public boolean orthogonalProjection(Point2DBasics pointToProject)
   {
      return orthogonalProjection(pointToProject, pointToProject);
   }

   /**
    * Computes the orthogonal projection of a 2D point on this 2D line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of this line segment is too small, i.e.
    * {@code this.lengthSquared() < Epsilons.ONE_TRILLIONTH}, this method returns
    * {@code firstEndpoint}.
    * <li>the projection can not be outside the line segment. When the projection on the
    * corresponding line is outside the line segment, the result is the closest of the two
    * endpoints.
    * </ul>
    * </p>
    * 
    * @param pointToProject the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto this line segment is
    *           stored. Modified.
    * @return whether the method succeeded or not.
    */
   public boolean orthogonalProjection(Point2DReadOnly pointToProject, Point2DBasics projectionToPack)
   {
      return EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(pointToProject, firstEndpoint, secondEndpoint, projectionToPack);
   }

   /**
    * Computes the coordinates of the point located at a given percentage on this line segment: <br>
    * {@code pointToPack.interpolate(firstEndpoint, secondEndpoint, percentage)} </br>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param percentage the percentage along this line segment of the point. Must be in [0, 1].
    * @return the computed point.
    * @throws {@link RuntimeException} if {@code percentage} &notin; [0, 1].
    */
   public Point2D pointBetweenEndpointsGivenPercentage(double percentage)
   {
      Point2D point = new Point2D();
      pointBetweenEndpointsGivenPercentage(percentage, point);
      return point;
   }

   /**
    * Computes the coordinates of the point located at a given percentage on this line segment: <br>
    * {@code pointToPack.interpolate(firstEndpoint, secondEndpoint, percentage)} </br>
    * 
    * @param percentage the percentage along this line segment of the point. Must be in [0, 1].
    * @param pointToPack where the result is stored. Modified.
    * @throws {@link RuntimeException} if {@code percentage} &notin; [0, 1].
    */
   public void pointBetweenEndpointsGivenPercentage(double percentage, Point2DBasics pointToPack)
   {
      if (percentage < 0.0 || percentage > 1.0)
         throw new RuntimeException("Percentage must be between 0.0 and 1.0. Was: " + percentage);

      pointToPack.interpolate(firstEndpoint, secondEndpoint, percentage);
   }

   /**
    * Computes the coordinates of the point located on the line this line segment is lying on: <br>
    * {@code pointToPack.interpolate(firstEndpoint, secondEndpoint, percentage)} </br>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param percentage the percentage along this line segment of the point.
    * @return the computed point.
    */
   public Point2D pointOnLineGivenPercentage(double percentage)
   {
      Point2D point = new Point2D();
      pointOnLineGivenPercentage(percentage, point);
      return point;
   }

   /**
    * Computes the coordinates of the point located on the line this line segment is lying on: <br>
    * {@code pointToPack.interpolate(firstEndpoint, secondEndpoint, percentage)} </br>
    * 
    * @param percentage the percentage along this line segment of the point.
    * @param pointToPack where the result is stored. Modified.
    */
   public void pointOnLineGivenPercentage(double percentage, Point2DBasics pointToPack)
   {
      pointToPack.interpolate(firstEndpoint, secondEndpoint, percentage);
   }

   /**
    * Computes and returns the coordinates of the point located exactly at the middle of this line
    * segment.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @return the mid-point of this line segment.
    */
   public Point2D midpoint()
   {
      Point2D midpoint = new Point2D();
      midpoint(midpoint);
      return midpoint;
   }

   /**
    * Computes the coordinates of the point located exactly at the middle of this line segment.
    * 
    * @param midpointToPack point in which the mid-point of this line segment is stored. Modified.
    */
   public void midpoint(Point2DBasics midpointToPack)
   {
      midpointToPack.interpolate(firstEndpoint, secondEndpoint, 0.5);
   }

   /**
    * Computes the vector going from the first to the second endpoint of this line segment.
    * 
    * @param normalize whether the direction vector is to be normalized.
    * @param directionToPack vector in which the direction is stored. Modified.
    */
   public void getDirection(boolean normalize, Vector2DBasics directionToPack)
   {
      directionToPack.sub(secondEndpoint, firstEndpoint);
      if (normalize)
         directionToPack.normalize();
   }

   /**
    * Computes the vector going from the first to the second endpoint of this line segment.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param normalize whether the direction vector is to be normalized.
    * @return the direction of this line segment.
    */
   public Vector2D getDirection(boolean normalize)
   {
      Vector2D direction = new Vector2D();
      getDirection(normalize, direction);
      return direction;
   }

   /**
    * Tests whether the projection of the given point onto this line segment is located between the
    * two endpoints or exactly on an endpoint.
    * 
    * @param point the query. Not modified.
    * @return {@code true} if the projection of the point is between the endpoints of this line
    *         segment, {@code false} otherwise.
    */
   public boolean isBetweenEndpoints(Point2DReadOnly point)
   {
      return isBetweenEndpoints(point, 0);
   }

   /**
    * Tests whether the projection of the given point onto this line segment is located between the
    * two endpoints with a given conservative tolerance {@code epsilon}:
    * <ul>
    * <li>if {@code epsilon > 0}, the point has to be between the endpoints and at a minimum
    * distance of {@code epsilon * this.length()} from the closest endpoint.
    * <li>if {@code epsilon < 0}, the point has to be between the endpoints or at a maximum distance
    * of {@code -epsilon * this.length()} from the closest endpoint.
    * <li>if {@code epsilon = 0}, the point has to be between the endpoints or equal to one of the
    * endpoints.
    * </ul>
    * 
    * @param point the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the projection of the point is between the endpoints of this line
    *         segment, {@code false} otherwise.
    */
   public boolean isBetweenEndpoints(Point2DReadOnly point, double epsilon)
   {
      return isBetweenEndpoints(point.getX(), point.getY(), epsilon);
   }

   /**
    * Tests whether the projection of the given point onto this line segment is located between the
    * two endpoints with a given conservative tolerance {@code epsilon}:
    * <ul>
    * <li>if {@code epsilon > 0}, the point has to be between the endpoints and at a minimum
    * distance of {@code epsilon * this.length()} from the closest endpoint.
    * <li>if {@code epsilon < 0}, the point has to be between the endpoints or at a maximum distance
    * of {@code -epsilon * this.length()} from the closest endpoint.
    * <li>if {@code epsilon = 0}, the point has to be between the endpoints or equal to one of the
    * endpoints.
    * </ul>
    * 
    * @param x the x-coordinate of the query point.
    * @param y the y-coordinate of the query point.
    * @param z the z-coordinate of the query point.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the projection of the point is between the endpoints of this line
    *         segment, {@code false} otherwise.
    */
   public boolean isBetweenEndpoints(double x, double y, double epsilon)
   {
      double alpha = percentageAlongLineSegment(x, y);

      if (alpha < epsilon)
         return false;
      if (alpha > 1.0 - epsilon)
         return false;

      return true;
   }

   /**
    * Computes a percentage along the line segment representing the location of the given point once
    * projected onto this line segment. The returned percentage is in ] -&infin;; &infin; [,
    * {@code 0.0} representing {@code firstEndpoint}, and {@code 1.0} representing
    * {@code secondEndpoint}.
    * <p>
    * For example, if the returned percentage is {@code 0.5}, it means that the projection of the
    * given point is located at the middle of this line segment. The coordinates of the projection
    * of the point can be computed from the {@code percentage} as follows: <code>
    * Point3D projection = new Point3D(); </br>
    * projection.interpolate(lineSegmentStart, lineSegmentEnd, percentage); </br>
    * </code>
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the given line segment is too small, i.e.
    * {@code this.lengthSquared() < Epsilons.ONE_TRILLIONTH}, this method fails and returns
    * {@code 0.0}.
    * </ul>
    * </p>
    * 
    * @param point the query point. Not modified.
    * @return the computed percentage along the line segment representing where the point projection
    *         is located.
    */
   public double percentageAlongLineSegment(Point2DReadOnly point)
   {
      return percentageAlongLineSegment(point.getX(), point.getY());
   }

   /**
    * Computes a percentage along the line segment representing the location of the given point once
    * projected onto this line segment. The returned percentage is in ] -&infin;; &infin; [,
    * {@code 0.0} representing {@code firstEndpoint}, and {@code 1.0} representing
    * {@code secondEndpoint}.
    * <p>
    * For example, if the returned percentage is {@code 0.5}, it means that the projection of the
    * given point is located at the middle of this line segment. The coordinates of the projection
    * of the point can be computed from the {@code percentage} as follows: <code>
    * Point3D projection = new Point3D(); </br>
    * projection.interpolate(lineSegmentStart, lineSegmentEnd, percentage); </br>
    * </code>
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the given line segment is too small, i.e.
    * {@code this.lengthSquared() < Epsilons.ONE_TRILLIONTH}, this method fails and returns
    * {@code 0.0}.
    * </ul>
    * </p>
    * 
    * @param x the x-coordinate of the query point.
    * @param y the y-coordinate of the query point.
    * @param z the z-coordinate of the query point.
    * @return the computed percentage along the line segment representing where the point projection
    *         is located.
    */
   public double percentageAlongLineSegment(double x, double y)
   {
      return EuclidGeometryTools.percentageAlongLineSegment2D(x, y, firstEndpoint, secondEndpoint);
   }

   /**
    * Swaps this line segment's endpoints.
    */
   public void flipDirection()
   {
      double x = firstEndpoint.getX();
      double y = firstEndpoint.getY();

      firstEndpoint.set(secondEndpoint);
      secondEndpoint.set(x, y);
   }

   public double dotProduct(LineSegment2d lineSegment2d)
   {
      return EuclidGeometryTools.dotProduct(firstEndpoint, secondEndpoint, lineSegment2d.firstEndpoint, lineSegment2d.secondEndpoint);
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

   /**
    * Gets the read-only reference to the first endpoint of this line segment.
    * 
    * @return the reference to the first endpoint of this line segment.
    */
   public Point2DReadOnly getFirstEndpoint()
   {
      return firstEndpoint;
   }

   /**
    * Gets the first endpoint defining this line segment by storing its coordinates in the given
    * argument {@code firstEndpointToPack}.
    * 
    * @param firstEndpointToPack point in which the coordinates of this line segment's first
    *           endpoint are stored. Modified.
    */
   public void getFirstEndpoint(Point2DBasics firstEndpointToPack)
   {
      firstEndpointToPack.set(firstEndpoint);
   }

   /**
    * Gets a copy of the first endpoint of this line segment.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @return the copy of the first endpoint of this line segment.
    */
   public Point2D getFirstEndpointCopy()
   {
      return new Point2D(firstEndpoint);
   }

   /**
    * Gets the read-only reference to the second endpoint of this line segment.
    * 
    * @return the reference to the second endpoint of this line segment.
    */
   public Point2DReadOnly getSecondEndpoint()
   {
      return secondEndpoint;
   }

   /**
    * Gets the second endpoint defining this line segment by storing its coordinates in the given
    * argument {@code secondEndpointToPack}.
    * 
    * @param secondEndpointToPack point in which the coordinates of this line segment's second
    *           endpoint are stored. Modified.
    */
   public void getSecondEndpoint(Point2DBasics secondEndpointToPack)
   {
      secondEndpointToPack.set(secondEndpoint);
   }

   /**
    * Gets a copy of the second endpoint of this line segment.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @return the copy of the second endpoint of this line segment.
    */
   public Point2D getSecondEndpointCopy()
   {
      return new Point2D(secondEndpoint);
   }

   /**
    * Gets the endpoints defining this line segment by storing their coordinates in the given
    * arguments.
    * 
    * @param firstEndpointToPack point in which the coordinates of this line segment's first
    *           endpoint are stored. Modified.
    * @param secondEndpointToPack point in which the coordinates of this line segment's second
    *           endpoint are stored. Modified.
    */
   public void getEndpoints(Point2DBasics firstEndpointToPack, Point2DBasics secondEndpointToPack)
   {
      firstEndpointToPack.set(firstEndpoint);
      secondEndpointToPack.set(secondEndpoint);
   }

   /**
    * Gets a copy of the endpoints of this line segment and returns them in a two-element array.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @return the two-element array containing in order this line segment first and second
    *         endpoints.
    */
   public Point2D[] getEndpointsCopy()
   {
      return new Point2D[] {getFirstEndpointCopy(), getSecondEndpointCopy()};
   }

   /**
    * Gets the x-coordinate of the first endpoint defining this line segment.
    * 
    * @return the first endpoint x-coordinate.
    */
   public double getFirstEndpointX()
   {
      return firstEndpoint.getX();
   }

   /**
    * Gets the y-coordinate of the first endpoint defining this line segment.
    * 
    * @return the first endpoint y-coordinate.
    */
   public double getFirstEndpointY()
   {
      return firstEndpoint.getY();
   }

   /**
    * Gets the x-coordinate of the second endpoint defining this line segment.
    * 
    * @return the first endpoint x-coordinate.
    */
   public double getSecondEndpointX()
   {
      return secondEndpoint.getX();
   }

   /**
    * Gets the y-coordinate of the second endpoint defining this line segment.
    * 
    * @return the first endpoint y-coordinate.
    */
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
