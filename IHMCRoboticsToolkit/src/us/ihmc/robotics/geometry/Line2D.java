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
 * Represents an infinitely-long 2D line defined by a 2D point and a 2D unit-vector.
 */
public class Line2D implements GeometryObject<Line2D>
{
   private final static double minAllowableVectorPart = Math.sqrt(Double.MIN_NORMAL);

   /** Coordinates of a point located on this line. */
   private final Point2D point = new Point2D();
   /** Normalized direction of this line. */
   private final Vector2D direction = new Vector2D();

   private boolean hasPointBeenSet = false;
   private boolean hasDirectionBeenSet = false;

   /**
    * Default constructor that initializes both {@link #point} and {@link #direction} to zero. This
    * point and vector have to be set to valid values to make this line usable.
    */
   public Line2D()
   {
      hasPointBeenSet = false;
      hasDirectionBeenSet = false;
   }

   /**
    * Initializes this line to be passing through the given point, with the vector as the direction.
    * 
    * @param pointOnLineX the x-coordinate of a point on this line.
    * @param pointOnLineY the y-coordinate of a point on this line.
    * @param lineDirectionX the x-component of the direction of this line.
    * @param lineDirectionY the y-component of the direction of this line.
    * @throws RuntimeException if the new direction is unreasonably small.
    */
   public Line2D(double pointOnLineX, double pointOnLineY, double lineDirectionX, double lineDirectionY)
   {
      set(pointOnLineX, pointOnLineY, lineDirectionX, lineDirectionY);
   }

   /**
    * Creates a new line 2D and initializes it to {@code other}.
    * 
    * @param other the other line used to initialize this line. Not modified.
    * @throws RuntimeException if the other line has not been initialized yet.
    */
   public Line2D(Line2D other)
   {
      set(other);
   }

   /**
    * Initializes this line to be passing through the two given points.
    * 
    * @param firstPointOnLine first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    * @throws RuntimeException if the new direction is unreasonably small.
    * @throws RuntimeException if the two given points are exactly equal.
    */
   public Line2D(Point2DReadOnly firstPointOnLine, Point2DReadOnly secondPointOnLine)
   {
      set(firstPointOnLine, secondPointOnLine);
   }

   /**
    * Initializes this line to be passing through the given point, with the vector as the direction.
    * 
    * @param pointOnLine point on this line. Not modified.
    * @param lineDirection direction of this line. Not modified.
    * @throws RuntimeException if the new direction is unreasonably small.
    */
   public Line2D(Point2DReadOnly pointOnLine, Vector2DReadOnly lineDirection)
   {
      set(pointOnLine, lineDirection);
   }

   /**
    * Transforms this line using the given homogeneous transformation matrix.
    * 
    * @param transform the transform to apply on this line's point and vector. Not modified.
    * @throws RuntimeException if this line has not been initialized yet.
    * @throws NotAMatrix2DException if the rotation part of {@code transform} is not a
    *            transformation in the XY-plane.
    */
   @Override
   public void applyTransform(Transform transform)
   {
      checkHasBeenInitialized();
      point.applyTransform(transform);
      direction.applyTransform(transform);
   }

   /**
    * Transforms this line using the given homogeneous transformation matrix and project the result
    * onto the XY-plane.
    * 
    * @param transform the transform to apply on this line's point and vector. Not modified.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public void applyTransformAndProjectToXYPlane(Transform transform)
   {
      checkHasBeenInitialized();
      point.applyTransform(transform, false);
      direction.applyTransform(transform, false);
   }

   /**
    * Copies this line, transforms the copy using the given homogeneous transformation matrix and
    * project the result onto the XY-plane, and returns the result.
    * 
    * @param transform the transform to apply on this line's copy. Not modified.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public Line2D applyTransformAndProjectToXYPlaneCopy(Transform transform)
   {
      Line2D copy = new Line2D(this);
      copy.applyTransformAndProjectToXYPlane(transform);
      return copy;
   }

   /**
    * Copies this line, transforms the copy using the given homogeneous transformation matrix, and
    * returns the result.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param transform the transform to apply on this line's copy. Not modified.
    * @throws RuntimeException if this line has not been initialized yet.
    * @throws NotAMatrix2DException if the rotation part of {@code transform} is not a
    *            transformation in the XY-plane.
    */
   public Line2D applyTransformCopy(Transform transform)
   {
      checkHasBeenInitialized();
      Line2D copy = new Line2D(this);
      copy.applyTransform(transform);
      return copy;
   }

   /**
    * Tests if this line and the other line are perpendicular.
    * 
    * @param other the query. Not modified.
    * @return {@code true} if the two lines are perpendicular, {@code false} otherwise.
    * @throws RuntimeException if either this line or {@code other} has not been initialized yet.
    */
   public boolean areLinesPerpendicular(Line2D other)
   {
      checkHasBeenInitialized();
      // Dot product of two vectors is zero if the vectors are perpendicular
      return direction.dot(other.getDirection()) < 1e-7;
   }

   private void checkDistinctPoints(Point2DReadOnly firstPointOnLine, Point2DReadOnly secondPointOnLine)
   {
      if (firstPointOnLine.equals(secondPointOnLine))
      {
         throw new RuntimeException("Tried to create a line from two coincidal points");
      }
   }

   private void checkHasBeenInitialized()
   {
      if (!hasPointBeenSet)
         throw new RuntimeException("The point of this line has not been initialized.");
      if (!hasDirectionBeenSet)
         throw new RuntimeException("The direction of this line has not been initialized.");
   }

   private void checkReasonableVector(Vector2DReadOnly localVector)
   {
      if (Math.abs(localVector.getX()) < minAllowableVectorPart && Math.abs(localVector.getY()) < minAllowableVectorPart)
      {
         throw new RuntimeException("Line length must be greater than zero");
      }
   }

   /**
    * Tests if this line contains {@link Double#NaN}.
    * 
    * @return {@code true} if {@link #point} and/or {@link #direction} contains {@link Double#NaN},
    *         {@code false} otherwise.
    */
   @Override
   public boolean containsNaN()
   {
      return point.containsNaN() || direction.containsNaN();
   }

   /**
    * Computes the minimum distance the given 3D point and this line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code direction.length() < }{@link EuclidGeometryTools#ONE_TRILLIONTH}, this method
    * returns the distance between {@code point} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param point 2D point to compute the distance from the line. Not modified.
    * @return the minimum distance between the 3D point and this 3D line.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public double distance(Point2DReadOnly point)
   {
      checkHasBeenInitialized();
      return EuclidGeometryTools.distanceFromPoint2DToLine2D(point, this.point, direction);
   }

   /**
    * Tests on a per-component basis on the point and vector if this line is equal to {@code other}
    * with the tolerance {@code epsilon}. This method will return {@code false} if the two lines are
    * physically the same but either the point or vector of each line is different. For instance, if
    * {@code this.point == other.point} and {@code this.direction == - other.direction}, the two
    * lines are physically the same but this method returns {@code false}.
    * 
    * @param other the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two lines are equal, {@code false} otherwise.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   @Override
   public boolean epsilonEquals(Line2D other, double epsilon)
   {
      checkHasBeenInitialized();
      if (!point.epsilonEquals(other.point, epsilon))
         return false;
      if (!direction.epsilonEquals(other.direction, epsilon))
         return false;

      return true;
   }

   /**
    * Tests on a per component basis, if this line 2D is exactly equal to {@code other}.
    *
    * @param other the other line 2D to compare against this. Not modified.
    * @return {@code true} if the two lines are exactly equal component-wise, {@code false}
    *         otherwise.
    */
   public boolean equals(Line2D other)
   {
      if (other == null)
         return false;
      else
         return point.equals(other.point) && direction.equals(other.direction);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method
    * returns {@link #equals(Line2D)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object obj)
   {
      try
      {
         return equals((Line2D) obj);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   /**
    * Gets the read-only reference to the direction of this line.
    * 
    * @return the reference to the direction.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public Vector2DReadOnly getDirection()
   {
      checkHasBeenInitialized();
      return direction;
   }

   /**
    * Gets the direction defining this line by storing its components in the given argument
    * {@code directionToPack}.
    * 
    * @param directionToPack vector in which the components of this line's direction are stored.
    *           Modified.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public void getDirection(Vector2DBasics directionToPack)
   {
      checkHasBeenInitialized();
      directionToPack.set(direction);
   }

   /**
    * Gets the x-component of this line's direction.
    * 
    * @return the x-component of this line's direction.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public double getDirectionX()
   {
      checkHasBeenInitialized();
      return direction.getX();
   }

   /**
    * Gets the y-component of this line's direction.
    * 
    * @return the y-component of this line's direction.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public double getDirectionY()
   {
      checkHasBeenInitialized();
      return direction.getY();
   }

   /**
    * Gets the read-only reference to the point through which this line is going.
    * 
    * @return the reference to the point.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public Point2DReadOnly getPoint()
   {
      checkHasBeenInitialized();
      return point;
   }

   /**
    * Gets the point defining this line by storing its coordinates in the given argument
    * {@code pointToPack}.
    * 
    * @param pointToPack point in which the coordinates of this line's point are stored. Modified.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public void getPoint(Point2DBasics pointOnLineToPack)
   {
      pointOnLineToPack.set(point);
   }

   /**
    * Gets the point and direction defining this line by storing their components in the given
    * arguments {@code pointToPack} and {@code directionToPack}.
    * 
    * @param pointToPack point in which the coordinates of this line's point are stored. Modified.
    * @param directionToPack vector in which the components of this line's direction are stored.
    *           Modified.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public void getPointAndDirection(Point2DBasics pointToPack, Vector2DBasics directionToPack)
   {
      getPoint(pointToPack);
      getDirection(directionToPack);
   }

   /**
    * Gets the x-coordinate of a point this line goes through.
    * 
    * @return the x-coordinate of this line's point.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public double getPointX()
   {
      checkHasBeenInitialized();
      return point.getX();
   }

   /**
    * Gets the y-coordinate of a point this line goes through.
    * 
    * @return the y-coordinate of this line's point.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public double getPointY()
   {
      checkHasBeenInitialized();
      return point.getY();
   }

   /**
    * Gets the coordinates of two distinct points this line goes through.
    * 
    * @param firstPointOnLineToPack the coordinates of a first point located on this line. Modified.
    * @param secondPointOnLineToPack the coordinates of a second point located on this line.
    *           Modified.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public void getTwoPointsOnLine(Point2DBasics firstPointOnLineToPack, Point2DBasics secondPointOnLineToPack)
   {
      checkHasBeenInitialized();
      firstPointOnLineToPack.set(point);
      secondPointOnLineToPack.add(point, direction);
   }

   /**
    * Calculates the interior bisector defined by this line and the given {@code secondLine}.
    * <p>
    * The interior bisector is defined as follows:
    * <ul>
    * <li>It goes through the intersection between this line and {@code secondLine}.
    * <li>Its direction point toward this line direction and the {@code secondLine}'s direction such
    * that: {@code interiorBisector.direction.dot(this.direction) > 0.0} and
    * {@code interiorBisector.direction.dot(secondLine.direction) > 0.0}.
    * <li>Finally the angle from {@code this} to the interior bisector is half the angle from
    * {@code this} to {@code secondLine}.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the two lines are parallel but not collinear, this method fails, returns {@code null}.
    * <li>If the two lines are collinear, this method returns a copy of {@code this}.
    * </ul>
    * </p>
    * 
    * @param secondLine the second line needed to calculate the interior bisector. Not modified.
    * @return the interior bisector if this method succeeded, {@code null} otherwise.
    * @throws RuntimeException if either this line or {@code secondLine} has not been initialized
    *            yet.
    */
   public Line2D interiorBisector(Line2D secondLine)
   {
      Line2D interiorBisector = new Line2D();
      boolean success = interiorBisector(secondLine, interiorBisector);
      return success ? interiorBisector : null;
   }

   /**
    * Calculates the interior bisector defined by this line and the given {@code secondLine}.
    * <p>
    * The interior bisector is defined as follows:
    * <ul>
    * <li>It goes through the intersection between this line and {@code secondLine}.
    * <li>Its direction point toward this line direction and the {@code secondLine}'s direction such
    * that: {@code interiorBisector.direction.dot(this.direction) > 0.0} and
    * {@code interiorBisector.direction.dot(secondLine.direction) > 0.0}.
    * <li>Finally the angle from {@code this} to the interior bisector is half the angle from
    * {@code this} to {@code secondLine}.
    * </ul>
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the two lines are parallel but not collinear, this method fails, returns {@code false},
    * and {@code interiorBisectorToPack} remains unchanged.
    * <li>If the two lines are collinear, {@code interiorBisectorToPack} is set to {@code this}.
    * </ul>
    * </p>
    * 
    * @param secondLine the second line needed to calculate the interior bisector. Not modified.
    * @param interiorBisectorToPack the line in which the interior bisector point and direction are
    *           stored. Modified.
    * @return {@code true} if this method succeeded, {@code false} otherwise.
    * @throws RuntimeException if either this line or {@code secondLine} has not been initialized
    *            yet.
    */
   public boolean interiorBisector(Line2D secondLine, Line2D interiorBisectorToPack)
   {
      checkHasBeenInitialized();
      secondLine.checkHasBeenInitialized();

      double t = EuclidGeometryTools.percentageOfIntersectionBetweenTwoLine2Ds(point, direction, secondLine.point, secondLine.direction);

      if (Double.isNaN(t))
      { // Lines are parallel but not collinear
         return false;
      }
      else if (t == 0.0 && EuclidGeometryTools.areVector2DsParallel(direction, secondLine.direction, 1.0e-7))
      { // Lines are collinear
         interiorBisectorToPack.set(this);
         return true;
      }
      else
      { // Lines are not parallel
         double pointOnBisectorX = t * direction.getX() + point.getX();
         double pointOnBisectorY = t * direction.getY() + point.getY();
         double bisectorDirectionX = direction.getX() + secondLine.direction.getX();
         double bisectorDirectionY = direction.getY() + secondLine.direction.getY();
         interiorBisectorToPack.set(pointOnBisectorX, pointOnBisectorY, bisectorDirectionX, bisectorDirectionY);
         return true;
      }
   }

   /**
    * Computes the coordinates of the possible intersection(s) between this line and the given
    * convex polygon 2D.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method behaves as if there is no intersections and
    * returns {@code null}.
    * <li>If no intersections exist, this method returns {@code null}.
    * </ul>
    * </p>
    * 
    * @param convexPolygon the convex polygon this line may intersect. Not modified.
    * @return the intersections between between the line and the polygon or {@code null} if the
    *         method failed or if there is no intersections.
    * @throws RuntimeException if this line has not been initialized yet.
    * @throws OutdatedPolygonException if the convex polygon is not up-to-date.
    */
   public Point2D[] intersectionWith(ConvexPolygon2d convexPolygon)
   {
      checkHasBeenInitialized();
      return convexPolygon.intersectionWith(this);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between this line and the given
    * convex polygon 2D.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method behaves as if there is no intersections.
    * <li>If no intersections exist, this method returns {@code 0} and the two intersection-to-pack
    * arguments remain unmodified.
    * <li>If there is only one intersection, this method returns {@code 1} and the coordinates of
    * the only intersection are stored in {@code firstIntersectionToPack}.
    * {@code secondIntersectionToPack} remains unmodified.
    * </ul>
    * </p>
    * 
    * @param convexPolygon the convex polygon this line may intersect. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection. Can
    *           be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection. Can
    *           be {@code null}. Modified.
    * @throws RuntimeException if this line has not been initialized yet.
    * @throws OutdatedPolygonException if the convex polygon is not up-to-date.
    */
   public int intersectionWith(ConvexPolygon2d convexPolygon, Point2DBasics firstIntersectionToPack, Point2DBasics secondIntersectionToPack)
   {
      checkHasBeenInitialized();
      return convexPolygon.intersectionWith(this, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Calculates the coordinates of the intersection between this line and the given line and
    * returns the result.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the two lines are parallel but not collinear, the two lines do not intersect and this
    * method returns {@code null}.
    * <li>if the two lines are collinear, the two lines are assumed to be intersecting at
    * {@code this.point}.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param secondLine the other line that may intersect this line. Not modified.
    * @return the coordinates of the intersection if the two lines intersects, {@code null}
    *         otherwise.
    * @throws RuntimeException if either this line or {@code secondLine} has not been initialized
    *            yet.
    */
   public Point2D intersectionWith(Line2D secondLine)
   {
      checkHasBeenInitialized();
      return EuclidGeometryTools.intersectionBetweenTwoLine2Ds(point, direction, secondLine.getPoint(), secondLine.getDirection());
   }

   /**
    * Calculates the coordinates of the intersection between this line and the given line and stores
    * the result in {@code intersectionToPack}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the two lines are parallel but not collinear, the two lines do not intersect and this
    * method returns {@code null}.
    * <li>if the two lines are collinear, the two lines are assumed to be intersecting at
    * {@code this.point}.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param secondLine the other line that may intersect this line. Not modified.
    * @param intersectionToPack the 2D point in which the result is stored. Modified.
    * @return {@code true} if the two lines intersects, {@code false} otherwise.
    * @throws RuntimeException if either this line or {@code secondLine} has not been initialized
    *            yet.
    */
   public boolean intersectionWith(Line2D secondLine, Point2DBasics intersectionToPack)
   {
      checkHasBeenInitialized();
      return EuclidGeometryTools.intersectionBetweenTwoLine2Ds(point, direction, secondLine.getPoint(), secondLine.getDirection(), intersectionToPack);
   }

   /**
    * Calculates the coordinates of the intersection between this line and the given line segment
    * and returns the result.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When this line and the line segment are parallel but not collinear, they do not intersect.
    * <li>When this line and the line segment are collinear, they are assumed to intersect at
    * {@code lineSegmentStart}.
    * <li>When this line intersects the line segment at one of its endpoints, this method returns a
    * copy of the endpoint where the intersection is happening.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param lineSegment the line segment that may intersect this line. Not modified.
    * @return the coordinates of the intersection if the line intersects the line segment,
    *         {@code null} otherwise.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public Point2D intersectionWith(LineSegment2d lineSegment)
   {
      checkHasBeenInitialized();
      return EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(point, direction, lineSegment.getFirstEndpoint(), lineSegment.getSecondEndpoint());
   }

   /**
    * Calculates the coordinates of the intersection between this line and the given line segment
    * and stores the result in {@code intersectionToPack}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When this line and the line segment are parallel but not collinear, they do not intersect.
    * <li>When this line and the line segment are collinear, they are assumed to intersect at
    * {@code lineSegmentStart}.
    * <li>When this line intersects the line segment at one of its endpoints, this method returns
    * {@code true} and the endpoint is the intersection.
    * </ul>
    * </p>
    * 
    * @param lineSegment the line segment that may intersect this line. Not modified.
    * @param intersectionToPack the 2D point in which the result is stored. Can be {@code null}.
    *           Modified.
    * @return {@code true} if the line intersects the line segment, {@code false} otherwise.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public boolean intersectionWith(LineSegment2d lineSegment, Point2DBasics intersectionToPack)
   {
      checkHasBeenInitialized();
      return EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(point, direction, lineSegment.getFirstEndpoint(), lineSegment.getSecondEndpoint(),
                                                                           intersectionToPack);
   }

   /**
    * Returns a boolean value, stating whether the query point is in behind of this line or not.
    * <p>
    * The idea of 'behind' refers to the side of the line the x-axis is pointing away.
    * </p>
    *
    * @param point the coordinates of the query. Not modified.
    * @return {@code true} if the point is in behind of this line, {@code false} if the point is
    *         front the line.
    * @throws RuntimeException if this line has not been initialized yet.
    * @throws RuntimeException if the given point is located exactly on this line.
    */
   public boolean isPointBehindLine(Point2DReadOnly point)
   {
      return !isPointInFrontOfLine(point);
   }

   /**
    * Returns a boolean value, stating whether the query point is in front of this line or not.
    * <p>
    * The idea of 'front' refers to the side of the line toward which the x-axis is pointing.
    * </p>
    *
    * @param point the coordinates of the query. Not modified.
    * @return {@code true} if the point is in front of this line, {@code false} if the point is
    *         behind the line.
    * @throws RuntimeException if this line has not been initialized yet.
    * @throws RuntimeException if the given point is located exactly on this line.
    */
   public boolean isPointInFrontOfLine(Point2DReadOnly point)
   {
      checkHasBeenInitialized();
      if (direction.getY() > 0.0)
         return isPointOnRightSideOfLine(point);
      else if (direction.getY() < 0.0)
         return isPointOnLeftSideOfLine(point);
      else
         throw new RuntimeException("Not defined when line is pointing exactly along the x-axis");
   }

   /**
    * Returns a boolean value, stating whether the query point is in front of this line or not.
    * <p>
    * The idea of 'front' refers to the side of the line toward which the given vector
    * {@code frontDirection} is pointing.
    * </p>
    *
    * @param frontDirection the vector used to define the side of the line which is to be considered
    *           as the front. Not modified.
    * @param point the coordinates of the query. Not modified.
    * @return {@code true} if the point is in front of this line, {@code false} if the point is
    *         behind the line.
    * @throws RuntimeException if this line has not been initialized yet.
    * @throws RuntimeException if the given point is located exactly on this line.
    */
   public boolean isPointInFrontOfLine(Vector2DReadOnly frontDirection, Point2DReadOnly point)
   {
      double crossProduct = frontDirection.cross(direction);
      if (crossProduct > 0.0)
         return isPointOnRightSideOfLine(point);
      else if (crossProduct < 0.0)
         return isPointOnLeftSideOfLine(point);
      else
         throw new RuntimeException("Not defined when line is pointing exactly along the front direction");
   }

   /**
    * Returns a boolean value, stating whether a 2D point is on the left or right side of this line.
    * The idea of "side" is determined based on the direction of the line.
    * <p>
    * For instance, given the {@code this.direction} components x = 0, and y = 1, and the
    * {@code this.point} coordinates x = 0, and y = 0, a point located on:
    * <ul>
    * <li>the left side of this line has a negative y coordinate.
    * <li>the right side of this line has a positive y coordinate.
    * </ul>
    * </p>
    * This method will return {@code false} if the point is on this line.
    *
    * @param point the coordinates of the query point.
    * @return {@code true} if the point is on the left side of this line, {@code false} if the point
    *         is on the right side or exactly on this line.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public boolean isPointOnLeftSideOfLine(Point2DReadOnly point)
   {
      return isPointOnSideOfLine(point, true);
   }

   /**
    * Tests if the given is located on this line.
    * <p>
    * More precisely, the point is assumed to be on this line if it is located at a distance less
    * than {@code 1.0e-8} from it.
    * </p>
    * 
    * @param point the coordinates of the query. Not modified.
    * @return {@code true} if the point is located on this line, {@code false} otherwise.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public boolean isPointOnLine(Point2DReadOnly point)
   {
      return isPointOnLine(point, 1.0e-8);
   }

   /**
    * Tests if the given is located on this line.
    * <p>
    * More precisely, the point is assumed to be on this line if it is located at a distance less
    * than {@code epsilon} from it.
    * </p>
    * 
    * @param point the coordinates of the query. Not modified.
    * @param epsilon the tolerance used for this test.
    * @return {@code true} if the point is located on this line, {@code false} otherwise.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public boolean isPointOnLine(Point2DReadOnly point, double epsilon)
   {
      checkHasBeenInitialized();
      return EuclidGeometryTools.distanceFromPoint2DToLine2D(point, this.point, direction) < epsilon;
   }

   /**
    * Returns a boolean value, stating whether a 2D point is on the left or right side of this line.
    * The idea of "side" is determined based on the direction of the line.
    * <p>
    * For instance, given the {@code this.direction} components x = 0, and y = 1, and the
    * {@code this.point} coordinates x = 0, and y = 0, a point located on:
    * <ul>
    * <li>the left side of this line has a negative y coordinate.
    * <li>the right side of this line has a positive y coordinate.
    * </ul>
    * </p>
    * This method will return {@code false} if the point is on this line.
    *
    * @param point the coordinates of the query point.
    * @return {@code true} if the point is on the right side of this line, {@code false} if the
    *         point is on the left side or exactly on this line.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public boolean isPointOnRightSideOfLine(Point2DReadOnly point)
   {
      return isPointOnSideOfLine(point, false);
   }

   /**
    * Returns a boolean value, stating whether a 2D point is on the left or right side of this line.
    * The idea of "side" is determined based on the direction of the line.
    * <p>
    * For instance, given the {@code this.direction} components x = 0, and y = 1, and the
    * {@code this.point} coordinates x = 0, and y = 0, a point located on:
    * <ul>
    * <li>the left side of this line has a negative y coordinate.
    * <li>the right side of this line has a positive y coordinate.
    * </ul>
    * </p>
    * This method will return {@code false} if the point is on this line.
    *
    * @param pointX the x-coordinate of the query point.
    * @param pointY the y-coordinate of the query point.
    * @param testLeftSide the query of the side, when equal to {@code true} this will test for the
    *           left side, {@code false} this will test for the right side.
    * @return {@code true} if the point is on the query side of this line, {@code false} if the
    *         point is on the opposite side or exactly on this line.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public boolean isPointOnSideOfLine(double pointX, double pointY, boolean testLeftSide)
   {
      checkHasBeenInitialized();
      return EuclidGeometryTools.isPoint2DOnSideOfLine2D(pointX, pointY, this.point, direction, testLeftSide);
   }

   /**
    * Returns a boolean value, stating whether a 2D point is on the left or right side of this line.
    * The idea of "side" is determined based on the direction of the line.
    * <p>
    * For instance, given the {@code this.direction} components x = 0, and y = 1, and the
    * {@code this.point} coordinates x = 0, and y = 0, a point located on:
    * <ul>
    * <li>the left side of this line has a negative y coordinate.
    * <li>the right side of this line has a positive y coordinate.
    * </ul>
    * </p>
    * This method will return {@code false} if the point is on this line.
    *
    * @param point the coordinates of the query point.
    * @param testLeftSide the query of the side, when equal to {@code true} this will test for the
    *           left side, {@code false} this will test for the right side.
    * @return {@code true} if the point is on the query side of this line, {@code false} if the
    *         point is on the opposite side or exactly on this line.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public boolean isPointOnSideOfLine(Point2DReadOnly point, boolean testLeftSide)
   {
      checkHasBeenInitialized();
      return EuclidGeometryTools.isPoint2DOnSideOfLine2D(point, this.point, direction, testLeftSide);
   }

   /**
    * Flips this line's direction.
    * 
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public void negateDirection()
   {
      checkHasBeenInitialized();
      direction.negate();
   }

   /**
    * Copies this line and then flips the direction of the copy before returning it.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public Line2D negateDirectionCopy()
   {
      checkHasBeenInitialized();
      Line2D ret = new Line2D(this);
      ret.negateDirection();

      return ret;
   }

   /**
    * Computes the orthogonal projection of the given 2D point on this 2D line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the given line direction is too small, i.e.
    * {@code lineDirection.lengthSquared() < }{@value #ONE_TRILLIONTH}, this method fails and
    * returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject the point to project on this line. Modified.
    * @return whether the method succeeded or not.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public boolean orthogonalProjection(Point2DBasics pointToProject)
   {
      return orthogonalProjection(pointToProject, pointToProject);
   }

   /**
    * Computes the orthogonal projection of the given 2D point on this 2D line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the given line direction is too small, i.e.
    * {@code lineDirection.lengthSquared() < }{@value #ONE_TRILLIONTH}, this method fails and
    * returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto the line is stored.
    *           Modified.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public boolean orthogonalProjection(Point2DReadOnly pointToProject, Point2DBasics projectionToPack)
   {
      checkHasBeenInitialized();
      return EuclidGeometryTools.orthogonalProjectionOnLine2D(pointToProject, point, direction, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of the given 2D point on this 2D line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the given line direction is too small, i.e.
    * {@code lineDirection.lengthSquared() < }{@value #ONE_TRILLIONTH}, this method fails and
    * returns {@code false}.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @return the projection of the point onto the line or {@code null} if the method failed.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public Point2D orthogonalProjectionCopy(Point2DReadOnly pointToProject)
   {
      checkHasBeenInitialized();
      return EuclidGeometryTools.orthogonalProjectionOnLine2D(pointToProject, point, direction);
   }

   /**
    * Calculates the parameter 't' corresponding to the coordinates of the given {@code pointOnLine}
    * 'p' by solving the line equation:<br>
    * p = t * n + p<sub>0</sub><br>
    * where n is the unit-vector defining the direction of this line and p<sub>0</sub> is the point
    * defining this line which also corresponds to the point for which t=0.
    * <p>
    * Note that the absolute value of 't' is equal to the distance between the point 'p' and the
    * point p<sub>0</sub> defining this line.
    * </p>
    * 
    * @param pointOnLine the coordinates of the 'p' from which the parameter 't' is to be
    *           calculated. The point has to be on the line. Not modified.
    * @param epsilon the maximum distance allowed between the given point and this line. If the
    *           given point is at a distance less than {@code epsilon} from this line, it is
    *           considered as being located on this line.
    * @return the value of the parameter 't' corresponding to the given point.
    * @throws RuntimeException if this line has not been initialized yet.
    * @throws RuntimeException if the given point is located at a distance greater than
    *            {@code epsilon} from this line.
    */
   public double parameterGivenPointOnLine(Point2DReadOnly pointOnLine, double epsilon)
   {
      if (!isPointOnLine(pointOnLine, epsilon))
      {
         throw new RuntimeException("The given point is not on this line, distance from line: " + distance(pointOnLine));
      }
      else
      {
         double x0 = this.point.getX();
         double y0 = this.point.getY();
         double x1 = x0 + direction.getX();
         double y1 = y0 + direction.getY();
         return EuclidGeometryTools.percentageAlongLineSegment2D(pointOnLine.getX(), pointOnLine.getY(), x0, y0, x1, y1);
      }
   }

   /**
    * Calculates and returns a line that is perpendicular to this line, with its direction pointing
    * to the left of this line, while going through the given point.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param point the point the line has to go through. Not modified.
    * @return the line perpendicular to {@code this} and going through {@code point}.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public Line2D perpendicularLineThroughPoint(Point2DReadOnly point)
   {
      checkHasBeenInitialized();
      return new Line2D(point, perpendicularVector());
   }

   /**
    * Modifies {@code perpendicularLineToPack} such that it is perpendicular to this line, with its
    * direction pointing to the left of this line, while going through the given point.
    * 
    * @param point the point the line has to go through. Not modified.
    * @param perpendicularLineToPack the line perpendicular to {@code this} and going through
    *           {@code point}. Modified.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public void perpendicularLineThroughPoint(Point2DReadOnly point, Line2D perpendicularLineToPack)
   {
      checkHasBeenInitialized();
      perpendicularLineToPack.set(point.getX(), point.getY(), -direction.getY(), direction.getX());
   }

   /**
    * Returns the vector that is perpendicular to this line and pointing to the left.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @return the perpendicular vector to this line.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public Vector2D perpendicularVector()
   {
      checkHasBeenInitialized();
      return EuclidGeometryTools.perpendicularVector2D(direction);
   }

   /**
    * Packs into {@code vectorToPack} the vector that is perpendicular to this line and pointing to
    * the left.
    * 
    * @param vectorToPack the perpendicular vector to this line. Modified.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public void perpendicularVector(Vector2DBasics vectorToPack)
   {
      checkHasBeenInitialized();
      EuclidGeometryTools.perpendicularVector2D(direction, vectorToPack);
   }

   /**
    * Calculates the coordinates of the point 'p' given the parameter 't' as follows:<br>
    * p = t * n + p<sub>0</sub><br>
    * where n is the unit-vector defining the direction of this line and p<sub>0</sub> is the point
    * defining this line which also corresponds to the point for which t=0.
    * <p>
    * Note that the absolute value of 't' is equal to the distance between the point 'p' and the
    * point p<sub>0</sub> defining this line.
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param t the parameter used to calculate the point coordinates.
    * @return the coordinates of the point 'p'.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public Point2D pointOnLineGivenParameter(double t)
   {
      Point2D pointToReturn = new Point2D();
      pointOnLineGivenParameter(t, pointToReturn);
      return pointToReturn;
   }

   /**
    * Calculates the coordinates of the point 'p' given the parameter 't' as follows:<br>
    * p = t * n + p<sub>0</sub><br>
    * where n is the unit-vector defining the direction of this line and p<sub>0</sub> is the point
    * defining this line which also corresponds to the point for which t=0.
    * <p>
    * Note that the absolute value of 't' is equal to the distance between the point 'p' and the
    * point p<sub>0</sub> defining this line.
    * </p>
    * 
    * @param t the parameter used to calculate the point coordinates.
    * @param pointToPack the point in which the coordinates of 'p' are stored. Modified.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public void pointOnLineGivenParameter(double t, Point2DBasics pointToPack)
   {
      checkHasBeenInitialized();
      pointToPack.scaleAdd(t, direction, point);
   }

   /**
    * Applies a counter-clockwise rotation to the direction of this line about the z-axis by
    * {@code angleInRadians}.
    * <p>
    * Note that the point of this line remains unchanged.
    * </p>
    * 
    * @param angleInRadians the angle to rotate this line's direction in radians.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public void rotate(double angleInRadians)
   {
      checkHasBeenInitialized();
      double vXOld = direction.getX();
      double vYOld = direction.getY();

      double vXNew = Math.cos(angleInRadians) * vXOld - Math.sin(angleInRadians) * vYOld;
      double vYNew = Math.sin(angleInRadians) * vXOld + Math.cos(angleInRadians) * vYOld;

      direction.set(vXNew, vYNew);
   }

   /**
    * Redefines this line with a new point and a new direction vector.
    * 
    * @param pointOnLineX the new x-coordinate of the point on this line.
    * @param pointOnLineY the new y-coordinate of the point on this line.
    * @param lineDirectionX the new x-component of the direction of this line.
    * @param lineDirectionY the new y-component of the direction of this line.
    * @throws RuntimeException if the new direction is unreasonably small.
    */
   public void set(double pointOnLineX, double pointOnLineY, double lineDirectionX, double lineDirectionY)
   {
      setPoint(pointOnLineX, pointOnLineY);
      setDirection(lineDirectionX, lineDirectionY);
   }

   /**
    * Sets this line to be the same as the given line.
    * 
    * @param other the other line to copy. Not modified.
    * @throws RuntimeException if the other line has not been initialized yet.
    */
   @Override
   public void set(Line2D other)
   {
      point.set(other.getPoint());
      direction.set(other.getDirection());
      hasPointBeenSet = true;
      hasDirectionBeenSet = true;
   }

   /**
    * Redefines this line such that it goes through the two given points.
    * 
    * @param firstPointOnLine first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    * @throws RuntimeException if the new direction is unreasonably small.
    * @throws RuntimeException if the two given points are exactly equal.
    */
   public void set(Point2DReadOnly firstPointOnLine, Point2DReadOnly secondPointOnLine)
   {
      checkDistinctPoints(firstPointOnLine, secondPointOnLine);
      setPoint(firstPointOnLine);
      setDirection(secondPointOnLine.getX() - firstPointOnLine.getX(), secondPointOnLine.getY() - firstPointOnLine.getY());
   }

   /**
    * Redefines this line with a new point and a new direction vector.
    * 
    * @param pointOnLine new point on this line. Not modified.
    * @param lineDirection new direction of this line. Not modified.
    * @throws RuntimeException if the new direction is unreasonably small.
    */
   public void set(Point2DReadOnly pointOnLine, Vector2DReadOnly lineDirection)
   {
      setPoint(pointOnLine);
      setDirection(lineDirection);
   }

   /**
    * Redefines this line such that it goes through the two given points.
    * 
    * @param twoPointsOnLine a two-element array containing in order the first point and second
    *           point this line is to go through. Not modified.
    * @throws RuntimeException if the new direction is unreasonably small.
    * @throws RuntimeException if the two given points are exactly equal.
    * @throws IllegalArgumentException if the given array has a length different than 2.
    */
   public void set(Point2DReadOnly[] twoPointsOnLine)
   {
      if (twoPointsOnLine.length != 2)
         throw new IllegalArgumentException("Length of input array is not correct. Length = " + twoPointsOnLine.length + ", expected an array of two elements");
      set(twoPointsOnLine[0], twoPointsOnLine[1]);
   }

   /**
    * Changes the direction of this line by setting it to the normalized value of the given vector.
    * 
    * @param directionX the new x-component of the direction of this line.
    * @param directionY the new y-component of the direction of this line.
    * @throws RuntimeException if the new direction is unreasonably small.
    */
   public void setDirection(double lineDirectionX, double lineDirectionY)
   {
      direction.set(lineDirectionX, lineDirectionY);
      checkReasonableVector(direction);
      direction.normalize();
      hasDirectionBeenSet = true;
   }

   /**
    * Changes the direction of this line by setting it to the normalized value of the given vector.
    * 
    * @param lineDirection new direction of this line. Not modified.
    * @throws RuntimeException if the new direction is unreasonably small.
    */
   public void setDirection(Vector2DReadOnly lineDirection)
   {
      setDirection(lineDirection.getX(), lineDirection.getY());
   }

   /**
    * Changes the point through which this line has to go.
    * 
    * @param pointX the new x-coordinate of the point on this line.
    * @param pointY the new y-coordinate of the point on this line.
    */
   public void setPoint(double pointOnLineX, double pointOnLineY)
   {
      point.set(pointOnLineX, pointOnLineY);
      hasPointBeenSet = true;
   }

   /**
    * Changes the point through which this line has to go.
    * 
    * @param pointOnLine new point on this line. Not modified.
    */
   public void setPoint(Point2DReadOnly pointOnLine)
   {
      setPoint(pointOnLine.getX(), pointOnLine.getY());
   }

   /**
    * Sets the point and vector of this line to {@link Double#NaN}. After calling this method, this
    * line becomes invalid. A new valid point and valid vector will have to be set so this line is
    * again usable.
    */
   @Override
   public void setToNaN()
   {
      point.setToNaN();
      direction.setToNaN();
   }

   /**
    * Sets the point and vector of this line to zero. After calling this method, this line becomes
    * invalid. A new valid point and valid vector will have to be set so this line is again usable.
    */
   @Override
   public void setToZero()
   {
      point.setToZero();
      direction.setToZero();
      hasPointBeenSet = false;
      hasDirectionBeenSet = false;
   }

   private void shift(boolean shiftToLeft, double distanceToShift)
   {
      checkHasBeenInitialized();
      double vectorX = direction.getX();
      double vectorY = direction.getY();

      double orthogonalVectorX = -vectorY;
      double orthogonalVectorY = vectorX;

      if (!shiftToLeft)
      {
         orthogonalVectorX = -orthogonalVectorX;
         orthogonalVectorY = -orthogonalVectorY;
      }

      orthogonalVectorX = distanceToShift * orthogonalVectorX;
      orthogonalVectorY = distanceToShift * orthogonalVectorY;

      translate(orthogonalVectorX, orthogonalVectorY);
   }

   /**
    * Translates this line by {@code distanceToShift} along the vector perpendicular to this line's
    * direction and pointing to the left.
    * <p>
    * Note that this line's direction remains unchanged.
    * </p>
    * 
    * @param distanceToShift the distance to shift this line.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public void shiftToLeft(double distanceToShift)
   {
      shift(true, distanceToShift);
   }

   /**
    * Translates this line by {@code distanceToShift} along the vector perpendicular to this line's
    * direction and pointing to the right.
    * <p>
    * Note that this line's direction remains unchanged.
    * </p>
    * 
    * @param distanceToShift the distance to shift this line.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public void shiftToRight(double distanceToShift)
   {
      shift(false, distanceToShift);
   }

   /**
    * Calculates the slope value of this line.
    * <p>
    * The slope 's' can be used to calculate the y-coordinate of a point located on the line given
    * its x-coordinate:<br>
    * y = s * x + y<sub>0</sub><br>
    * where y<sub>0</sub> is the y-coordinate at which this line intercepts the y-axis and which can
    * be obtained with {@link #yIntercept()}.
    * </p>
    * 
    * @return the value of the slope of this line.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public double slope()
   {
      checkHasBeenInitialized();
      if (direction.getX() == 0.0 && direction.getY() > 0.0)
      {
         return Double.POSITIVE_INFINITY;
      }

      if (direction.getX() == 0.0 && direction.getY() < 0.0)
      {
         return Double.NEGATIVE_INFINITY;
      }

      return direction.getY() / direction.getX();
   }

   /**
    * Provides a {@code String} representation of this line 2D as follows:<br>
    * Line 2D: point = (x, y), direction = (x, y)
    *
    * @return the {@code String} representing this line 2D.
    */
   @Override
   public String toString()
   {
      return "Line 2D: point = " + point + ", direction = " + direction;
   }

   /**
    * Translates this line by the given (x, y).
    * <p>
    * Note that this line's direction remains unchanged.
    * </p>
    * 
    * @param x the distance to translate this line along the x-axis.
    * @param y the distance to translate this line along the y-axis.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public void translate(double x, double y)
   {
      checkHasBeenInitialized();
      point.add(x, y);
   }

   /**
    * The x-coordinate at which this line intercept the x-axis, i.e. the line defined by
    * {@code y=0}.
    * 
    * @return the x-coordinate of the intersection between this line and the x-axis.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public double xIntercept()
   {
      checkHasBeenInitialized();
      double parameterAtIntercept = -point.getY() / direction.getY();
      return parameterAtIntercept * direction.getX() + point.getX();
   }

   /**
    * The y-coordinate at which this line intercept the y-axis, i.e. the line defined by
    * {@code x=0}.
    * 
    * @return the y-coordinate of the intersection between this line and the y-axis.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public double yIntercept()
   {
      checkHasBeenInitialized();
      double parameterAtIntercept = -point.getX() / direction.getX();
      return parameterAtIntercept * direction.getY() + point.getY();
   }
}
