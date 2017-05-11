package us.ihmc.robotics.geometry;

import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.MathTools;

/**
 * Represents a finite-length 1D line segment defined by its two 1D endpoints.
 */
public class LineSegment1D
{
   /** The first endpoint defining this line segment. */
   private double firstEndpoint = Double.NaN;
   /** The second endpoint defining this line segment. */
   private double secondEndpoint = Double.NaN;
   private boolean positiveDirection;

   /**
    * Default constructor that initializes both endpoints of this line segment to
    * {@link Double#NaN}.
    */
   public LineSegment1D()
   {
   }

   /**
    * Initializes this line segment to have the given endpoints.
    * 
    * @param firstEndpoint coordinate of the first endpoint of this line segment.
    * @param secondEndpoint coordinate of the second endpoint of this line segment.
    */
   public LineSegment1D(double firstEndpoint, double secondEndpoint)
   {
      set(firstEndpoint, secondEndpoint);
   }

   /**
    * Initializes this line segment to have the given endpoints.
    * 
    * @param endpoints a two-element array containing in order the first and second endpoints for
    *           this line segment. Not modified.
    * @throws IllegalArgumentException if the given array has a length different than 2.
    */
   public LineSegment1D(double[] endpoints)
   {
      set(endpoints);
   }

   /**
    * Computes the overlap between this line segment and the other line segment and stores the
    * result in {@code intersectionToPack}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the two line segments do not overlap, this method fails, returns {@code false} and
    * {@code intersecitonToPack} remains unchanged.
    * </ul>
    * </p>
    * 
    * @param other the other line segment. Not modified.
    * @param intersectionToPack line segment used to store the result. Modified.
    * @return whether the two line segment overlap or no.
    */
   public boolean computeOverlap(LineSegment1D other, LineSegment1D intersectionToPack)
   {
      if (!isOverlappingInclusive(other))
         return false;

      double intersectionMin = Math.max(getMinPoint(), other.getMinPoint());
      double intersectionMax = Math.min(getMaxPoint(), other.getMaxPoint());
      intersectionToPack.set(intersectionMin, intersectionMax);
      return true;
   }

   /**
    * Computes the overlap between this line segment and the other line segment and stores the
    * result in {@code intersectionToPack}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the two line segments do not overlap, this method fails, returns {@code false} and
    * {@code intersecitonToPack} remains unchanged.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param other the other line segment. Not modified.
    * @return the line segment representing the overlap, or {@code null} if the line segments do not
    *         overlap.
    */
   public LineSegment1D computeOverlap(LineSegment1D other)
   {
      if (!isOverlappingInclusive(other))
         return null;

      double intersectionMin = Math.max(getMinPoint(), other.getMinPoint());
      double intersectionMax = Math.min(getMaxPoint(), other.getMaxPoint());
      return new LineSegment1D(intersectionMin, intersectionMax);
   }

   /**
    * Tests if this line segment and the other overlap or not.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the two line segments only share one of their endpoint, they are considered to be
    * overlapping.
    * </ul>
    * </p>
    * 
    * @param other the query. Not modified.
    * @return {@code true} if the two line segments overlap, {@code false} otherwise.
    */
   public boolean isOverlappingInclusive(LineSegment1D other)
   {
      return isBetweenEndpointsInclusive(other.firstEndpoint) || isBetweenEndpointsInclusive(other.secondEndpoint)
            || other.isBetweenEndpointsInclusive(this.firstEndpoint) || other.isBetweenEndpointsInclusive(this.secondEndpoint);
   }

   /**
    * Tests if this line segment and the other overlap or not.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the two line segments only share one of their endpoint, they do not overlap.
    * </ul>
    * </p>
    * 
    * @param other the query. Not modified.
    * @return {@code true} if the two line segments overlap, {@code false} otherwise.
    */
   public boolean isOverlappingExclusive(LineSegment1D other)
   {
      return isBetweenEndpointsExclusive(other.firstEndpoint) || isBetweenEndpointsExclusive(other.secondEndpoint)
            || other.isBetweenEndpointsExclusive(this.firstEndpoint) || other.isBetweenEndpointsExclusive(this.secondEndpoint);
   }

   /**
    * Tests if this line segment entirely contains the given line segment {@code other}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the two line segments have one or two endpoints in common, this line segment entirely
    * contains {@code other}.
    * </ul>
    * </p>
    * 
    * @param other the query. Not modified.
    * @return {@code true} if this line segment contains {@code other}, {@code false} otherwise.
    */
   public boolean isBetweenEndpointsInclusive(LineSegment1D other)
   {
      return isBetweenEndpointsInclusive(other.firstEndpoint) && isBetweenEndpointsInclusive(other.secondEndpoint);
   }

   /**
    * Tests if this line segment entirely contains the given line segment {@code other}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the two line segments have one or two endpoints in common, this line segment does not
    * contain {@code other}.
    * </ul>
    * </p>
    * 
    * @param other the query. Not modified.
    * @return {@code true} if this line segment contains {@code other}, {@code false} otherwise.
    */
   public boolean isBetweenEndpointsExclusive(LineSegment1D other)
   {
      return isBetweenEndpointsExclusive(other.firstEndpoint) && isBetweenEndpointsExclusive(other.secondEndpoint);
   }

   /**
    * Tests if this line segment contains the given {@code point}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the given point is equal to one of this line segment's endpoints, the point is
    * considered to be between this line segment endpoints.
    * </ul>
    * </p>
    * 
    * @param point the query.
    * @return {@code true} if this line segment contains {@code point}, {@code false} otherwise.
    */
   public boolean isBetweenEndpointsInclusive(double point)
   {
      return MathTools.intervalContains(point, getMinPoint(), getMaxPoint());
   }

   /**
    * Tests if this line segment contains the given {@code point}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the given point is equal to one of this line segment's endpoints, the point is
    * considered to be outside this line segment.
    * </ul>
    * </p>
    * 
    * @param point the query.
    * @return {@code true} if this line segment contains {@code point}, {@code false} otherwise.
    */
   public boolean isBetweenEndpointsExclusive(double point)
   {
      return MathTools.intervalContains(point, getMinPoint(), getMaxPoint(), false, false);
   }

   /**
    * Tests whether the given point is located between the two endpoints with a given conservative
    * tolerance {@code epsilon}:
    * <p>
    * <li>if {@code epsilon > 0}, the point has to be between the endpoints and at a minimum
    * distance of {@code epsilon * this.length()} from the closest endpoint.
    * <li>if {@code epsilon < 0}, the point has to be between the endpoints or at a maximum distance
    * of {@code -epsilon * this.length()} from the closest endpoint.
    * <li>if {@code epsilon = 0}, the point has to be between the endpoints or equal to one of the
    * endpoints.
    * </p>
    * 
    * @param point the query.
    * @param epsilon the tolerance to use for this test. If positive, the test becomes less
    *           restrictive. If negative, the test becomes more conservative.
    * @return {@code true} if the point is considered to be between this line segment's endpoints,
    *         {@code false} otherwise.
    */
   public boolean isBetweenEndpoints(double point, double epsilon)
   {
      double alpha = (point - this.firstEndpoint) / (this.secondEndpoint - this.firstEndpoint);

      if (alpha < epsilon)
         return false;
      if (alpha > 1.0 - epsilon)
         return false;

      return true;
   }

   /**
    * Calculates the distance between the query and the closest endpoint.
    * <p>
    * If the point is located between the endpoints of this line segment, the returned value of the
    * distance is negative.
    * </p>
    * 
    * @param point the query.
    * @return the distance between the query and this line segment.
    */
   public double signedDistance(double point)
   {
      if (isBetweenEndpointsInclusive(point))
         return -Math.min(point - getMinPoint(), getMaxPoint() - point);
      else if (point < getMinPoint())
         return getMinPoint() - point;
      else
         return point - getMaxPoint();
   }

   /**
    * Calculates the distance between the query and the closest endpoint.
    * <p>
    * If the point is located between the endpoints of this line segment, this method returns 0.
    * </p>
    * 
    * @param point the query.
    * @return the distance between the query and this line segment.
    */
   public double distance(double point)
   {
      if (isBetweenEndpointsInclusive(point))
         return 0;
      else if (point < getMinPoint())
         return getMinPoint() - point;
      else
         return point - getMaxPoint();
   }

   /**
    * Extends this line segment to include the given point.
    * <p>
    * If the point is between this line segment's endpoints, this line segment remains unchanged.
    * </p>
    * 
    * @param point the point to be included by extended this line segment.
    */
   public void extendSegmentToPoint(double point)
   {
      if (isBetweenEndpointsInclusive(point))
         return;
      if (point < getMinPoint())
         setMinPoint(point);
      else
         setMaxPoint(point);
   }

   /**
    * Tests if the given point is located outside this line segment and that this line segment is
    * point away from the point.
    * 
    * @param point the query.
    * @return {@code true} if the point is located before this line segment, {@code false} if the
    *         point is located between the endpoints of this line segment or if this line segment is
    *         pointed toward the point.
    */
   public boolean isBefore(double point)
   {
      if (positiveDirection)
         return point < this.firstEndpoint;
      else
         return point > this.firstEndpoint;
   }

   /**
    * Tests if the given point is located outside this line segment and that this line segment is
    * point toward the point.
    * 
    * @param point the query.
    * @return {@code true} if the point is located after this line segment, {@code false} if the
    *         point is located between the endpoints of this line segment or if this line segment is
    *         pointed away from the point.
    */
   public boolean isAfter(double point)
   {
      if (positiveDirection)
         return point > this.secondEndpoint;
      else
         return point < this.secondEndpoint;
   }

   /**
    * Sets this line segment's endpoints.
    * 
    * @param firstEndpoint coordinate of the first endpoint.
    * @param secondEndpoint coordinate of the second endpoint.
    */
   public void set(double firstEndpoint, double secondEndpoint)
   {
      this.firstEndpoint = firstEndpoint;
      this.secondEndpoint = secondEndpoint;
      updateDirection();
   }

   /**
    * Redefines this line segment with new endpoints.
    * 
    * @param endpoints a two-element array containing in order the first and second endpoints for
    *           this line segment. Not modified.
    * @throws IllegalArgumentException if the given array has a length different than 2.
    */
   public void set(double[] endpoints)
   {
      if (endpoints.length != 2)
         throw new RuntimeException("Length of input array is not correct. Length = " + endpoints.length + ", expected an array of two elements");

      this.firstEndpoint = endpoints[0];
      this.secondEndpoint = endpoints[1];
      updateDirection();
   }

   /**
    * Sets the first endpoint of this line segment.
    * 
    * @param firstEndpoint coordinate of the first endpoint.
    */
   public void setFirstEndpoint(double firstEndpoint)
   {
      this.firstEndpoint = firstEndpoint;
      updateDirection();
   }

   /**
    * Sets the second endpoint of this line segment.
    * 
    * @param secondEndpoint coordinate of the second endpoint.
    */
   public void setSecondEndpoint(double secondEndpoint)
   {
      this.secondEndpoint = secondEndpoint;
      updateDirection();
   }

   /**
    * Updates the endpoint of this line segment with the lowest coordinate.
    * 
    * @param newMinPoint the new coordinate for the endpoint with the lowest coordinate.
    * @throws RuntimeException if the argument is greater or equal to the endpoint with greatest
    *            coordinate. This prevents to flip this line segment's direction.
    */
   public void setMinPoint(double newMinPoint)
   {
      if (newMinPoint >= getMaxPoint())
         throw new RuntimeException("Unexpected newMinPoint: " + newMinPoint + ", expected it to be less than the current max point: " + getMaxPoint());
      if (positiveDirection)
         this.firstEndpoint = newMinPoint;
      else
         this.secondEndpoint = newMinPoint;
      updateDirection();
   }

   /**
    * Updates the endpoint of this line segment with the highest coordinate.
    * 
    * @param newMinPoint the new coordinate for the endpoint with the highest coordinate.
    * @throws RuntimeException if the argument is less or equal to the endpoint with lowest
    *            coordinate. This prevents to flip this line segment's direction.
    */
   public void setMaxPoint(double newMaxPoint)
   {
      if (newMaxPoint <= getMinPoint())
         throw new RuntimeException("Unexpected newMaxPoint: " + newMaxPoint + ", expected it to be greater than the current min point: " + getMinPoint());
      if (positiveDirection)
         this.secondEndpoint = newMaxPoint;
      else
         this.firstEndpoint = newMaxPoint;
      updateDirection();
   }

   /**
    * Gets the coordinate of this line segment's first endpoint.
    * 
    * @return the first endpoint coordinate.
    */
   public double getFirstEndpoint()
   {
      return this.firstEndpoint;
   }

   /**
    * Gets the coordinate of this line segment's second endpoint.
    * 
    * @return the second endpoint coordinate.
    */
   public double getSecondEndpoint()
   {
      return this.secondEndpoint;
   }

   /**
    * Gets the highest coordinate of this line segment.
    * 
    * @return the endpoint with the maximum coordinate value.
    */
   public double getMaxPoint()
   {
      return positiveDirection ? this.secondEndpoint : this.firstEndpoint;
   }

   /**
    * Gets the lowest coordinate of this line segment.
    * 
    * @return the endpoint with the minimum coordinate value.
    */
   public double getMinPoint()
   {
      return positiveDirection ? this.firstEndpoint : this.secondEndpoint;
   }

   /**
    * Calculates the coordinate of the point located in the middle of this line segment.
    * 
    * @return the coordinate of the middle of this line segment.
    */
   public double getMidPoint()
   {
      return 0.5 * (this.firstEndpoint + this.secondEndpoint);
   }

   /**
    * Calculates the (absolute) length of this line segment.
    * 
    * @return the value of this line segment's length.
    */
   public double length()
   {
      return (positiveDirection ? 1.0 : -1.0) * (this.secondEndpoint - this.firstEndpoint);
   }

   /**
    * Compute the 3D equivalent of this line segment. The 3D equivalent of each end point is
    * computed as follows: {@code endPoint3d = endPoint1d * lineDirection3d + lineStart3d}.
    * 
    * @param line3d the 3D line used as reference to compute the 3D line segment.
    * @return the 3D equivalent of this line segment.
    */
   public LineSegment3D toLineSegment3d(Line3D line3d)
   {
      return toLineSegment3d(line3d.getPoint(), line3d.getDirection());
   }

   /**
    * Compute the 3D equivalent of this line segment. The 3D equivalent of each end point is
    * computed as follows: {@code endPoint3d = endPoint1d * direction3d + zero3d}.
    * 
    * @param zero3d position of the 3D equivalent of an endpoint equal to zero.
    * @param direction3d direction toward greater values of {@code endPoint1d}.
    * @return the 3D equivalent of this line segment.
    */
   public LineSegment3D toLineSegment3d(Point3DReadOnly zero3d, Vector3DReadOnly direction3d)
   {
      Point3D firstEndpoint = new Point3D();
      Point3D secondEndpoint = new Point3D();
      firstEndpoint.scaleAdd(this.firstEndpoint, direction3d, zero3d);
      secondEndpoint.scaleAdd(this.secondEndpoint, direction3d, zero3d);
      return new LineSegment3D(firstEndpoint, secondEndpoint);
   }

   /**
    * Compute the 2D equivalent of this line segment. The 2D equivalent of each end point is
    * computed as follows: {@code endPoint2d = endPoint1d * direction2d + zero2d}.
    * 
    * @param zero2d position of the 2D equivalent of an endpoint equal to zero.
    * @param direction2d direction toward greater values of {@code endPoint1d}.
    * @return the 2D equivalent of this line segment.
    */
   public LineSegment2D toLineSegment2d(Point2DReadOnly zero2d, Vector2DReadOnly direction2d)
   {
      Point2D firstEndpoint = new Point2D();
      Point2D secondEndpoint = new Point2D();
      firstEndpoint.scaleAdd(this.firstEndpoint, direction2d, zero2d);
      secondEndpoint.scaleAdd(this.secondEndpoint, direction2d, zero2d);
      return new LineSegment2D(firstEndpoint, secondEndpoint);
   }

   /**
    * Provides a {@code String} representation of this line segment 1D as follows:<br>
    * Line segment 1D: 1st endpoint = (x), 2nd endpoint = (x)
    *
    * @return the {@code String} representing this line segment 1D.
    */
   @Override
   public String toString()
   {
      return "Line segment 1D: 1st endpoint = (" + firstEndpoint + "), 2nd endpoint = (" + secondEndpoint + ")";
   }

   /**
    * Update the direction based on the endpoints relative locations.
    */
   private void updateDirection()
   {
      positiveDirection = this.firstEndpoint <= this.secondEndpoint;
   }
}
