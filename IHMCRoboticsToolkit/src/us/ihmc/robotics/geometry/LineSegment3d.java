package us.ihmc.robotics.geometry;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.interfaces.GeometryObject;
import us.ihmc.robotics.geometry.transformables.TransformablePoint3d;

/**
 * Represents a finite-length 3D line segment defined by its two 3D endpoints.
 * 
 * @author Sylvain Bertrand
 *
 */
public class LineSegment3d implements GeometryObject<LineSegment3d>
{
   private final TransformablePoint3d firstEndpoint = new TransformablePoint3d();
   private final TransformablePoint3d secondEndpoint = new TransformablePoint3d();

   /**
    * Default constructor that initializes both endpoints of this line segment to zero.
    */
   public LineSegment3d()
   {
   }

   /**
    * Initializes this line segment to have the given endpoints.
    * 
    * @param firstEndpoint the first endpoint of this line segment. Not modified.
    * @param secondEndpoint the second endpoint of this line segment. Not modified.
    */
   public LineSegment3d(Point3d firstEndpoint, Point3d secondEndpoint)
   {
      set(firstEndpoint, secondEndpoint);
   }

   /**
    * Changes the first endpoint of this line segment.
    * 
    * @param firstEndpointX x-coordinate of the new first endpoint.
    * @param firstEndpointY y-coordinate of the new first endpoint.
    * @param firstEndpointZ z-coordinate of the new first endpoint.
    */
   public void setFirstEndpoint(double firstEndpointX, double firstEndpointY, double firstEndpointZ)
   {
      firstEndpoint.set(firstEndpointX, firstEndpointY, firstEndpointZ);
   }

   /**
    * Changes the first endpoint of this line segment.
    * 
    * @param firstEndpoint new endpoint of this line segment. Not modified
    */
   public void setFirstEndpoint(Point3d firstEndpoint)
   {
      this.firstEndpoint.set(firstEndpoint);
   }

   /**
    * Changes the second endpoint of this line segment.
    * 
    * @param secondEndpointX x-coordinate of the new second endpoint.
    * @param secondEndpointY y-coordinate of the new second endpoint.
    * @param secondEndpointZ z-coordinate of the new second endpoint.
    */
   public void setSecondEndpoint(double secondEndpointX, double secondEndpointY, double secondEndpointZ)
   {
      secondEndpoint.set(secondEndpointX, secondEndpointY, secondEndpointZ);
   }

   /**
    * Changes the second endpoint of this line segment.
    * 
    * @param secondEndpoint new second endpoint of this line segment. Not modified.
    */
   public void setSecondEndpoint(Point3d secondEndpoint)
   {
      this.secondEndpoint.set(secondEndpoint);
   }

   /**
    * Redefines this line segments with new endpoints.
    * 
    * @param firstEndpointX x-coordinate of the new first endpoint.
    * @param firstEndpointY y-coordinate of the new first endpoint.
    * @param firstEndpointZ z-coordinate of the new first endpoint.
    * @param secondEndpointX x-coordinate of the new second endpoint.
    * @param secondEndpointY y-coordinate of the new second endpoint.
    * @param secondEndpointZ z-coordinate of the new second endpoint.
    */
   public void set(double firstEndpointX, double firstEndpointY, double firstEndpointZ, double secondEndpointX, double secondEndpointY, double secondEndpointZ)
   {
      setFirstEndpoint(firstEndpointX, firstEndpointY, firstEndpointZ);
      setSecondEndpoint(secondEndpointX, secondEndpointY, secondEndpointZ);
   }

   /**
    * Redefines this line segment with new endpoints.
    * 
    * @param firstEndpoint new endpoint of this line segment. Not modified
    * @param secondEndpoint new second endpoint of this line segment. Not modified.
    */
   public void set(Point3d firstEndpoint, Point3d secondEndpoint)
   {
      setFirstEndpoint(firstEndpoint);
      setSecondEndpoint(secondEndpoint);
   }

   /**
    * Redefines this line segment with a new first endpoint and a vector going
    * from the first to the second endpoint.
    * 
    * @param firstEndpoint new first endpoint. Not modified.
    * @param fromFirstToSecondEndpoint vector going from the first to the second endpoint. Not modified.
    */
   public void set(Point3d firstEndpoint, Vector3d fromFirstToSecondEndpoint)
   {
      this.firstEndpoint.set(firstEndpoint);
      this.secondEndpoint.add(firstEndpoint, fromFirstToSecondEndpoint);
   }

   /**
    * Sets this line segment to be same as the given line segment.
    * 
    * @param other the other line segment to copy. Not modified.
    */
   @Override
   public void set(LineSegment3d other)
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
    * Sets both endpoints of this line segment to {@link Double#NaN}.
    * After calling this method, this line segment becomes invalid.
    * A new pair of valid endpoints will have to be set so this line segment is again usable.
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
    * @return {@code true} if {@link #firstEndpoint} and/or {@link #secondEndpoint} contains {@link Double#NaN}, {@code false} otherwise.
    */
   @Override
   public boolean containsNaN()
   {
      return firstEndpoint.containsNaN() || secondEndpoint.containsNaN();
   }

   /**
    * Test if the first endpoint of this line segment contains {@link Double#NaN}.
    * 
    * @return {@code true} if {@link #firstEndpoint} contains {@link Double#NaN}, {@code false} otherwise.
    */
   public boolean firstEndpointContainsNaN()
   {
      return firstEndpoint.containsNaN();
   }

   /**
    * Test if the second endpoint of this line segment contains {@link Double#NaN}.
    * 
    * @return {@code true} if {@link #secondEndpoint} contains {@link Double#NaN}, {@code false} otherwise.
    */
   public boolean secondEndpointContainsNaN()
   {
      return secondEndpoint.containsNaN();
   }

   /**
    * Transforms this line segment using the given homogeneous transformation matrix.
    * 
    * @param transform the transform to apply on the endpoints of this line segment. Not modified.
    */
   @Override
   public void applyTransform(RigidBodyTransform transform)
   {
      transform.transform(firstEndpoint);
      transform.transform(secondEndpoint);
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
    * Returns the minimum distance between a point and this given line segment.
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if {@code this.length() < Epsilons.ONE_TRILLIONTH}, this method returns the distance between {@code firstEndpoint} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param point 3D point to compute the distance from this line segment. Not modified.
    * @return the minimum distance between the 3D point and this 3D line segment.
    */
   public double distance(Point3d point)
   {
      return GeometryTools.distanceFromPointToLineSegment(point, firstEndpoint, secondEndpoint);
   }

   /**
    * This methods computes the minimum distance between this line segment and the given one.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    * 
    * @param otherLineSegment the other line segment to compute the distance from. Not modified.
    * @return the minimum distance between the two line segments.
    */
   public double distance(LineSegment3d otherLineSegment)
   {
      return GeometryTools.distanceBetweenTwoLineSegments(firstEndpoint, secondEndpoint, otherLineSegment.firstEndpoint, otherLineSegment.secondEndpoint);
   }

   /**
    * Computes the orthogonal projection of a 3D point on this 3D line segment.
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if the length of this line segment is too small,
    *     i.e. {@code this.length() < Epsilons.ONE_TRILLIONTH},
    *      this method returns {@code firstEndpoint}.
    *    <li> the projection can not be outside the line segment.
    *     When the projection on the corresponding line is outside the line segment, the result is the closest of the two endpoints.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param pointToProject the point to compute the projection of. Not modified.
    * @return the projection of the point onto the line segment or {@code null} if the method failed.
    */
   public Point3d orthogonalProjectionCopy(Point3d pointToProject)
   {
      return GeometryTools.getOrthogonalProjectionOnLineSegment(pointToProject, firstEndpoint, secondEndpoint);
   }

   /**
    * Computes the orthogonal projection of a 3D point on this 3D line segment.
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if the length of this line segment is too small,
    *     i.e. {@code this.length() < Epsilons.ONE_TRILLIONTH},
    *      this method returns {@code firstEndpoint}.
    *    <li> the projection can not be outside the line segment.
    *     When the projection on the corresponding line is outside the line segment, the result is the closest of the two endpoints.
    * </ul>
    * </p>
    * 
    * @param pointToProject the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto this line segment is stored. Modified.
    * @return whether the method succeeded or not.
    */
   public boolean orthogonalProjection(Point3d pointToProject, Point3d projectionToPack)
   {
      return GeometryTools.getOrthogonalProjectionOnLineSegment(pointToProject, firstEndpoint, secondEndpoint, projectionToPack);
   }

   /**
    * Computes the coordinates of the point located at a given percentage on this line segment:
    * <br> {@code pointToPack.interpolate(firstEndpoint, secondEndpoint, percentage)} </br>
    * 
    * @param percentage the percentage along this line segment of the point. Must be in [0, 1].
    * @param pointToPack where the result is stored. Modified.
    * @thows {@link RuntimeException} if {@code percentage} &notin; [0, 1].
    */
   public void getPointAlongPercentageOfLineSegment(double percentage, Point3d pointToPack)
   {
      if (percentage < 0.0 || percentage > 1.0)
         throw new RuntimeException("Percentage must be between 0.0 and 1.0. Was: " + percentage);

      pointToPack.interpolate(firstEndpoint, secondEndpoint, percentage);
   }

   /**
    * Computes the coordinates of the point located exactly at the middle of this line segment.
    * 
    * @param midpointToPack point in which the mid-point of this line segment is stored. Modified.
    */
   public void getMidpoint(Point3d midpointToPack)
   {
      midpointToPack.interpolate(firstEndpoint, secondEndpoint, 0.5);
   }

   /**
    * Computes the vector going from the first to the second endpoint of this line segment.
    * 
    * @param normalize whether the direction vector is to be normalized.
    * @param directionToPack vector in which the direction is stored. Modified.
    */
   public void getDirection(boolean normalize, Vector3d directionToPack)
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
   public Vector3d getDirectionCopy(boolean normalize)
   {
      Vector3d direction = new Vector3d();
      getDirection(normalize, direction);
      return direction;
   }

   /**
    * Tests whether the projection of the given point onto this line segment is located
    * between the two endpoints with a given conservative tolerance {@code epsilon}:
    * <ul>
    *    <li> if {@code epsilon > 0}, the point has to be between the endpoints and at a minimum
    *     distance of {@code epsilon} from the closest endpoint.
    *    <li> if {@code epsilon < 0}, the point has to be between the endpoints or at a maximum
    *     distance of {@code epsilon} from the closest endpoint.
    *    <li> if (@code epsilon == 0}, the point has to be between the endpoints or equal to
    *     one of the endpoints.
    * </ul>
    * 
    * @param point the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the projection of the point is between the endpoints of this line segment, {@code false} otherwise.
    */
   public boolean isBetweenEndpoints(Point3d point, double epsilon)
   {
      return isBetweenEndpoints(point.getX(), point.getY(), point.getZ(), epsilon);
   }

   /**
    * Tests whether the projection of the given point onto this line segment is located
    * between the two endpoints with a given conservative tolerance {@code epsilon}:
    * <ul>
    *    <li> if {@code epsilon > 0}, the point has to be between the endpoints and at a minimum
    *     distance of {@code epsilon} from the closest endpoint.
    *    <li> if {@code epsilon < 0}, the point has to be between the endpoints or at a maximum
    *     distance of {@code epsilon} from the closest endpoint.
    *    <li> if (@code epsilon == 0}, the point has to be between the endpoints or equal to
    *     one of the endpoints.
    * </ul>
    * 
    * @param x the x-coordinate of the query point.
    * @param y the y-coordinate of the query point.
    * @param z the z-coordinate of the query point.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the projection of the point is between the endpoints of this line segment, {@code false} otherwise.
    */
   public boolean isBetweenEndpoints(double x, double y, double z, double epsilon)
   {
      double alpha = percentageAlongLineSegment(x, y, z);

      if (alpha < epsilon)
         return false;
      if (alpha > 1.0 - epsilon)
         return false;

      return true;
   }

   /**
    * Computes a percentage along the line segment representing the location of the given point once projected onto this line segment.
    * The returned percentage is in ] -&infin;; &infin; [, {@code 0.0} representing {@code firstEndpoint}, and {@code 1.0} representing {@code secondEndpoint}.
    * <p>
    * For example, if the returned percentage is {@code 0.5}, it means that the projection of the given point is located at the middle of this line segment.
    * The coordinates of the projection of the point can be computed from the {@code percentage} as follows:
    * <code>
    * Point3d projection = new Point3d(); </br>
    * projection.interpolate(lineSegmentStart, lineSegmentEnd, percentage); </br>
    * </code>
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if the length of the given line segment is too small, i.e. {@code this.length() < Epsilons.ONE_TRILLIONTH}, this method fails and returns {@code 0.0}.
    * </ul>
    * </p>
    * 
    * @param point the query point. Not modified.
    * @return the computed percentage along the line segment representing where the point projection is located.
    */
   public double percentageAlongLineSegment(Point3d point)
   {
      return percentageAlongLineSegment(point.getX(), point.getY(), point.getZ());
   }

   /**
    * Computes a percentage along the line segment representing the location of the given point once projected onto this line segment.
    * The returned percentage is in ] -&infin;; &infin; [, {@code 0.0} representing {@code firstEndpoint}, and {@code 1.0} representing {@code secondEndpoint}.
    * <p>
    * For example, if the returned percentage is {@code 0.5}, it means that the projection of the given point is located at the middle of this line segment.
    * The coordinates of the projection of the point can be computed from the {@code percentage} as follows:
    * <code>
    * Point3d projection = new Point3d(); </br>
    * projection.interpolate(lineSegmentStart, lineSegmentEnd, percentage); </br>
    * </code>
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if the length of the given line segment is too small, i.e. {@code this.length() < Epsilons.ONE_TRILLIONTH}, this method fails and returns {@code 0.0}.
    * </ul>
    * </p>
    * 
    * @param x the x-coordinate of the query point.
    * @param y the y-coordinate of the query point.
    * @param z the z-coordinate of the query point.
    * @return the computed percentage along the line segment representing where the point projection is located.
    */
   public double percentageAlongLineSegment(double x, double y, double z)
   {
      return GeometryTools.getPercentageAlongLineSegment(x, y, z, firstEndpoint.getX(), firstEndpoint.getY(), firstEndpoint.getZ(), secondEndpoint.getX(),
                                                         secondEndpoint.getY(), secondEndpoint.getZ());
   }

   /**
    * @return the reference to the first endpoint of this line segment.
    */
   public Point3d getFirstEndpoint()
   {
      return firstEndpoint;
   }

   /**
    * @return the reference to the second endpoint of this line segment.
    */
   public Point3d getSecondEndpoint()
   {
      return secondEndpoint;
   }

   /**
    * Computes the line on which this line segment is lying.
    * The line's vector is the direction from the first to the second endpoint
    * of this line segment.
    * 
    * @param lineToPack the line on which this line segment is lying.
    */
   public void getLine(Line3d lineToPack)
   {
      lineToPack.set(firstEndpoint, secondEndpoint);
   }

   /**
    * Computes the line on which this line segment is lying.
    * The line's vector is the direction from the first to the second endpoint
    * of this line segment.
    * 
    * @return the line on which this line segment is lying.
    */
   public Line3d getLineCopy()
   {
      return new Line3d(firstEndpoint, secondEndpoint);
   }

   /**
    * Tests on a per-component basis on both endpoints if this line segment is equal to {@code other} with the tolerance {@code epsilon}.
    * 
    * @param other the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two line segments are equal, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(LineSegment3d other, double epsilon)
   {
      return firstEndpoint.epsilonEquals(other.firstEndpoint, epsilon) && secondEndpoint.epsilonEquals(other.secondEndpoint, epsilon);
   }
}
