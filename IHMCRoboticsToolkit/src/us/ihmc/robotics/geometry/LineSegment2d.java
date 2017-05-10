package us.ihmc.robotics.geometry;

import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.robotics.geometry.ConvexPolygonTools.OutdatedPolygonException;

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
    * @param endpoints a two-element array containing in order the first and second endpoints for
    *           this line segment. Not modified.
    * @throws IllegalArgumentException if the given array has a length different than 2.
    */
   public LineSegment2d(Point2DReadOnly[] endpoints)
   {
      set(endpoints);
   }

   /**
    * Transforms this line segment using the given homogeneous transformation matrix.
    * 
    * @param transform the transform to apply on the endpoints of this line segment. Not modified.
    * @throws NotAMatrix2DException if the rotation part of {@code transform} is not a
    *            transformation in the XY-plane.
    */
   @Override
   public void applyTransform(Transform transform)
   {
      firstEndpoint.applyTransform(transform);
      secondEndpoint.applyTransform(transform);
   }

   /**
    * Transforms this line segment using the given homogeneous transformation matrix and project the
    * result onto the XY-plane.
    * 
    * @param transform the transform to apply on this line segment's endpoints. Not modified.
    */
   public void applyTransformAndProjectToXYPlane(Transform transform)
   {
      firstEndpoint.applyTransform(transform, false);
      secondEndpoint.applyTransform(transform, false);
   }

   /**
    * Copies this line segment, transforms the copy using the given homogeneous transformation
    * matrix and project the result onto the XY-plane, and returns the result.
    * 
    * @param transform the transform to apply on this line segment's copy. Not modified.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public LineSegment2d applyTransformAndProjectToXYPlaneCopy(Transform transform)
   {
      LineSegment2d copy = new LineSegment2d(this);
      copy.applyTransformAndProjectToXYPlane(transform);
      return copy;
   }

   /**
    * Copies this line segment, transforms the copy using the given homogeneous transformation
    * matrix, and returns the result.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param transform the transform to apply on this line segment's copy. Not modified.
    * @throws NotAMatrix2DException if the rotation part of {@code transform} is not a
    *            transformation in the XY-plane.
    */
   public LineSegment2d applyTransformCopy(Transform transform)
   {
      LineSegment2d copy = new LineSegment2d(this);
      copy.applyTransform(transform);
      return copy;
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
    * Computes the vector going from the first to the second endpoint of this line segment.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param normalize whether the direction vector is to be normalized.
    * @return the direction of this line segment.
    */
   public Vector2D direction(boolean normalize)
   {
      Vector2D direction = new Vector2D();
      direction(normalize, direction);
      return direction;
   }

   /**
    * Computes the vector going from the first to the second endpoint of this line segment.
    * 
    * @param normalize whether the direction vector is to be normalized.
    * @param directionToPack vector in which the direction is stored. Modified.
    */
   public void direction(boolean normalize, Vector2DBasics directionToPack)
   {
      directionToPack.sub(secondEndpoint, firstEndpoint);
      if (normalize)
         directionToPack.normalize();
   }

   /**
    * Returns the minimum distance between a point and this given line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code this.lengthSquared() < }{@link EuclidGeometryTools#ONE_TRILLIONTH}, this method
    * returns the distance between {@code firstEndpoint} and the given {@code point}.
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
    * Returns the square of the minimum distance between a point and this given line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code this.lengthSquared() < }{@link EuclidGeometryTools#ONE_TRILLIONTH}, this method
    * returns the distance between {@code firstEndpoint} and the given {@code point}.
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
    * Computes the dot product of this line segment with the other line segment such that:<br>
    * {@code this }&middot;
    * {@code other = Math.cos(}&alpha;{@code ) * this.length() * other.length()}<br>
    * where &alpha; is the angle from this to the other line segment.
    * 
    * @param other the other line segment used to compute the dot product. Not modified.
    * @return the value of the dot product.
    */
   public double dotProduct(LineSegment2d other)
   {
      return EuclidGeometryTools.dotProduct(firstEndpoint, secondEndpoint, other.firstEndpoint, other.secondEndpoint);
   }

   /**
    * Tests on a per-component basis on both endpoints if this line segment is equal to
    * {@code other} with the tolerance {@code epsilon}.
    * 
    * @param other the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two line segments are equal, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(LineSegment2d other, double epsilon)
   {
      return firstEndpoint.epsilonEquals(other.firstEndpoint, epsilon) && secondEndpoint.epsilonEquals(other.secondEndpoint, epsilon);
   }

   /**
    * Tests on a per component basis, if this line segment 2D is exactly equal to {@code other}.
    *
    * @param other the other line segment 2D to compare against this. Not modified.
    * @return {@code true} if the two line segments are exactly equal component-wise, {@code false}
    *         otherwise.
    */
   public boolean equals(LineSegment2d other)
   {
      if (other == null)
         return false;
      else
         return firstEndpoint.equals(other.firstEndpoint) && secondEndpoint.equals(other.secondEndpoint);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method
    * returns {@link #equals(LineSegment2d)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object obj)
   {
      try
      {
         return equals((LineSegment2d) obj);
      }
      catch (ClassCastException e)
      {
         return false;
      }
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
    * Swaps this line segment's endpoints.
    */
   public void flipDirection()
   {
      double x = firstEndpoint.getX();
      double y = firstEndpoint.getY();

      firstEndpoint.set(secondEndpoint);
      secondEndpoint.set(x, y);
   }

   /**
    * Returns a copy of this line segment with the endpoints swapped.
    */
   public LineSegment2d flipDirectionCopy()
   {
      return new LineSegment2d(secondEndpoint, firstEndpoint);
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

   public Point2D[] intersectionWith(ConvexPolygon2d convexPolygon)
   {
      return convexPolygon.intersectionWith(this);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between this line segment and the
    * given convex polygon 2D.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method behaves as if there is no intersections.
    * <li>If no intersections exist, this method returns {@code 0} and the two intersection-to-pack
    * arguments remain unmodified.
    * <li>If there is only one intersection, this method returns {@code 1} and the coordinates of
    * the only intersection are stored in {@code firstIntersectionToPack}.
    * {@code secondIntersectionToPack} remains unmodified.
    * <li>If this line segment is collinear to an edge:
    * <ul>
    * <li>The edge entirely contains this line segment: this method finds two intersections which
    * are the endpoints of this line segment.
    * <li>This line segment entirely contains the edge: this method finds two intersections which
    * are the vertices of the edge.
    * <li>The edge and this line segment partially overlap: this method finds two intersections
    * which the polygon's vertex that on this line segment and this line segment's endpoint that is
    * on the polygon's edge.
    * </ul>
    * </ul>
    * </p>
    * 
    * @param convexPolygon the convex polygon this line segment may intersect. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection. Can
    *           be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection. Can
    *           be {@code null}. Modified.
    * @return the number of intersections between this line segment and the polygon.
    * @throws OutdatedPolygonException if the convex polygon is not up-to-date.
    */
   public int intersectionWith(ConvexPolygon2d convexPolygon, Point2DBasics firstIntersectionToPack, Point2DBasics secondIntersectionToPack)
   {
      return convexPolygon.intersectionWith(this, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Calculates the coordinates of the intersection between this line segment and the given line
    * and returns the result.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When this line segment and the line are parallel but not collinear, they do not intersect.
    * <li>When this line segment and the line are collinear, they are assumed to intersect at
    * {@code lineSegmentStart}.
    * <li>When the line intersects this line segment at one of its endpoints, this method returns
    * {@code true} and the endpoint is the intersection.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param line the line that may intersect this line segment. Not modified.
    * @return the coordinates of the intersection if the line intersects this line segment,
    *         {@code null} otherwise.
    * @return {@code true} if the line intersects this line segment, {@code false} otherwise.
    */
   public Point2D intersectionWith(Line2D line)
   {
      return EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(line.getPoint(), line.getDirection(), firstEndpoint, secondEndpoint);
   }

   /**
    * Calculates the coordinates of the intersection between this line segment and the given line
    * and stores the result in {@code intersectionToPack}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When this line segment and the line are parallel but not collinear, they do not intersect.
    * <li>When this line segment and the line are collinear, they are assumed to intersect at
    * {@code lineSegmentStart}.
    * <li>When the line intersects this line segment at one of its endpoints, this method returns
    * {@code true} and the endpoint is the intersection.
    * </ul>
    * </p>
    * 
    * @param line the line that may intersect this line segment. Not modified.
    * @param intersectionToPack the 2D point in which the result is stored. Can be {@code null}.
    *           Modified.
    * @return {@code true} if the line intersects this line segment, {@code false} otherwise.
    */
   public boolean intersectionWith(Line2D line, Point2DBasics intersectionToPack)
   {
      return EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(line.getPoint(), line.getDirection(), firstEndpoint, secondEndpoint,
                                                                           intersectionToPack);
   }

   /**
    * Computes the intersection between this line segment and the given line segment and returns the
    * result.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the two line segments are parallel but not collinear, the two line segments do not
    * intersect, this method returns {@code null}.
    * <li>When the two line segments are collinear, if the two line segments do not overlap do not
    * have at least one common endpoint, this method returns {@code null}.
    * <li>When the two line segments have a common endpoint, this method returns the common endpoint
    * as the intersection.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param secondLineSegment the other line segment that may intersect this line segment. Not
    *           modified.
    * @return the intersection point if it exists, {@code null} otherwise.
    */
   public Point2D intersectionWith(LineSegment2d secondLineSegment)
   {
      return EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(firstEndpoint, secondEndpoint, secondLineSegment.firstEndpoint,
                                                                      secondLineSegment.secondEndpoint);
   }

   /**
    * Computes the intersection between this line segment and the given line segment and stores the
    * result in {@code intersectionToPack}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the two line segments are parallel but not collinear, the two line segments do not
    * intersect.
    * <li>When the two line segments are collinear, this methods returns {@code true} only if the
    * two line segments overlap or have at least one common endpoint.
    * <li>When the two line segments have a common endpoint, this method returns true.
    * </ul>
    * </p>
    *
    * @param secondLineSegment the other line segment that may intersect this line segment. Not
    *           modified.
    * @param intersectionToPack the 2D point in which the result is stored. Modified.
    * @return {@code true} if the two lines intersects, {@code false} otherwise.
    */
   public boolean intersectionWith(LineSegment2d secondLineSegment, Point2D intersectionToPack)
   {
      return EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(firstEndpoint, secondEndpoint, secondLineSegment.firstEndpoint,
                                                                      secondLineSegment.secondEndpoint, intersectionToPack);
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
    * Returns a boolean value, stating whether a 2D point is on the left or right side of this line
    * segment. The idea of "side" is determined based on the direction of the line segment.
    * <p>
    * For instance, given the {@code this.firstEndpoint = (0, 0)} and
    * {@code this.secondEndpoint = (0, 1)}:
    * <ul>
    * <li>the left side of this line segment has a negative y coordinate.
    * <li>the right side of this line segment has a positive y coordinate.
    * </ul>
    * </p>
    * This method will return {@code false} if the point is on this line.
    *
    * @param point the coordinates of the query point.
    * @return {@code true} if the point is on the left side of this line segment, {@code false} if
    *         the point is on the right side or exactly on this line segment.
    */
   public boolean isPointOnLeftSideOfLineSegment(Point2DReadOnly point)
   {
      return EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D(point, firstEndpoint, secondEndpoint);
   }

   /**
    * Tests if the given is located on this line segment.
    * <p>
    * More precisely, the point is assumed to be on this line segment if it is located at a distance
    * less than {@code 1.0e-8} from it.
    * </p>
    * 
    * @param point the coordinates of the query. Not modified.
    * @return {@code true} if the point is located on this line segment, {@code false} otherwise.
    */
   public boolean isPointOnLineSegment(Point2DReadOnly point)
   {
      return isPointOnLineSegment(point, 1.0e-8);
   }

   /**
    * Tests if the given is located on this line segment.
    * <p>
    * More precisely, the point is assumed to be on this line segment if it is located at a distance
    * less than {@code epsilon} from it.
    * </p>
    * 
    * @param point the coordinates of the query. Not modified.
    * @param epsilon the tolerance used for this test.
    * @return {@code true} if the point is located on this line segment, {@code false} otherwise.
    */
   public boolean isPointOnLineSegment(Point2DReadOnly point, double epsilon)
   {
      return EuclidGeometryTools.distanceFromPoint2DToLineSegment2D(point, firstEndpoint, secondEndpoint) < epsilon;
   }

   /**
    * Returns a boolean value, stating whether a 2D point is on the left or right side of this line
    * segment. The idea of "side" is determined based on the direction of the line segment.
    * <p>
    * For instance, given the {@code this.firstEndpoint = (0, 0)} and
    * {@code this.secondEndpoint = (0, 1)}:
    * <ul>
    * <li>the left side of this line segment has a negative y coordinate.
    * <li>the right side of this line segment has a positive y coordinate.
    * </ul>
    * </p>
    * This method will return {@code false} if the point is on this line.
    *
    * @param point the coordinates of the query point.
    * @return {@code true} if the point is on the right side of this line segment, {@code false} if
    *         the point is on the left side or exactly on this line segment.
    */
   public boolean isPointOnRightSideOfLineSegment(Point2DReadOnly point)
   {
      return EuclidGeometryTools.isPoint2DOnRightSideOfLine2D(point, firstEndpoint, secondEndpoint);
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
    * Computes the orthogonal projection of a 2D point on this 2D line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of this line segment is too small, i.e.
    * {@code this.lengthSquared() < }{@link EuclidGeometryTools#ONE_TRILLIONTH}, this method returns
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
    * {@code this.lengthSquared() < }{@link EuclidGeometryTools#ONE_TRILLIONTH}, this method returns
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
    * Computes the orthogonal projection of a 2D point on this 2D line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of this line segment is too small, i.e.
    * {@code this.lengthSquared() < }{@link EuclidGeometryTools#ONE_TRILLIONTH}, this method returns
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
    * Computes a percentage along the line segment representing the location of the given point once
    * projected onto this line segment. The returned percentage is in ] -&infin;; &infin; [,
    * {@code 0.0} representing {@code firstEndpoint}, and {@code 1.0} representing
    * {@code secondEndpoint}.
    * <p>
    * For example, if the returned percentage is {@code 0.5}, it means that the projection of the
    * given point is located at the middle of this line segment. The coordinates of the projection
    * of the point can be computed from the {@code percentage} as follows:<br>
    * <code>
    * Point3D projection = new Point3D(); </br>
    * projection.interpolate(lineSegmentStart, lineSegmentEnd, percentage); </br>
    * </code>
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the given line segment is too small, i.e.
    * {@code this.lengthSquared() < }{@link EuclidGeometryTools#ONE_TRILLIONTH}, this method fails
    * and returns {@code 0.0}.
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
    * {@code this.lengthSquared() < }{@link EuclidGeometryTools#ONE_TRILLIONTH}, this method fails
    * and returns {@code 0.0}.
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
    * Computes the vector perpendicular to the direction of this line segment.
    * 
    * @param normalize whether the perpendicular vector is to be normalized.
    * @param perpendicularVectorToPack vector in which the perpendicular vector components are
    *           stored. Modified.
    */
   public void perpendicular(boolean normalize, Vector2DBasics perpendicularVectorToPack)
   {
      direction(normalize, perpendicularVectorToPack);
      EuclidGeometryTools.perpendicularVector2D(perpendicularVectorToPack, perpendicularVectorToPack);
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
    * Sets both endpoints of this line segment to zero.
    */
   @Override
   public void setToZero()
   {
      firstEndpoint.setToZero();
      secondEndpoint.setToZero();
   }

   private void shift(boolean shiftToLeft, double distanceToShift)
   {
      double vectorX = secondEndpoint.getX() - firstEndpoint.getX();
      double vectorY = secondEndpoint.getY() - firstEndpoint.getY();

      double length = length();
      double orthogonalVectorX = -vectorY / length;
      double orthogonalVectorY = vectorX / length;

      if (!shiftToLeft)
      {
         orthogonalVectorX = -orthogonalVectorX;
         orthogonalVectorY = -orthogonalVectorY;
      }

      orthogonalVectorX = distanceToShift * orthogonalVectorX;
      orthogonalVectorY = distanceToShift * orthogonalVectorY;

      translate(orthogonalVectorX, orthogonalVectorY);
   }

   private LineSegment2d shiftAndCopy(boolean shiftToLeft, double distanceToShift)
   {
      LineSegment2d shifted = new LineSegment2d(this);
      shifted.shift(shiftToLeft, distanceToShift);
      return shifted;
   }

   /**
    * Translates this line segment by {@code distanceToShift} along the vector perpendicular to this
    * line segment's direction and pointing to the left.
    * <p>
    * Note that the length and direction of this line segment remains unchanged.
    * </p>
    * 
    * @param distanceToShift the distance to shift this line segment.
    */
   public void shiftToLeft(double distanceToShift)
   {
      shift(true, distanceToShift);
   }

   /**
    * Copies this and translates the copy by {@code distanceToShift} along the vector perpendicular
    * to this line segment's direction and pointing to the left.
    * <p>
    * Note that the length and direction of the copy are the same as this line segment.
    * </p>
    * 
    * @param distanceToShift the distance to shift this line segment.
    * @return the shifted line segment.
    */
   public LineSegment2d shiftToLeftCopy(double distanceToShift)
   {
      return shiftAndCopy(true, distanceToShift);
   }

   /**
    * Translates this line segment by {@code distanceToShift} along the vector perpendicular to this
    * line segment's direction and pointing to the right.
    * <p>
    * Note that the length and direction of this line segment remains unchanged.
    * </p>
    * 
    * @param distanceToShift the distance to shift this line segment.
    */
   public void shiftToRight(double distanceToShift)
   {
      shift(false, distanceToShift);
   }

   /**
    * Copies this and translates the copy by {@code distanceToShift} along the vector perpendicular
    * to this line segment's direction and pointing to the right.
    * <p>
    * Note that the length and direction of the copy are the same as this line segment.
    * </p>
    * 
    * @param distanceToShift the distance to shift this line segment.
    * @return the shifted line segment.
    */
   public LineSegment2d shiftToRightCopy(double distanceToShift)
   {
      return shiftAndCopy(false, distanceToShift);
   }

   /**
    * Provides a {@code String} representation of this line segment 2D as follows:<br>
    * Line segment 2D: 1st endpoint = (x, y), 2nd endpoint = (x, y)
    *
    * @return the {@code String} representing this line segment 2D.
    */
   @Override
   public String toString()
   {
      return "Line segment 2D: 1st endpoint = " + firstEndpoint + ", 2nd endpoint = " + secondEndpoint;
   }

   /**
    * Translates this line segment by the given (x, y).
    * <p>
    * Note that the length and direction of this line segment remains unchanged.
    * </p>
    * 
    * @param x the distance to translate this line along the x-axis.
    * @param y the distance to translate this line along the y-axis.
    */
   public void translate(double x, double y)
   {
      firstEndpoint.add(x, y);
      secondEndpoint.add(x, y);
   }

}
