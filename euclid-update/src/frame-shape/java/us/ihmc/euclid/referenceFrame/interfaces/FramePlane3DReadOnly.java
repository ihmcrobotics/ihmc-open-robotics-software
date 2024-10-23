package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.Plane3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public interface FramePlane3DReadOnly extends Plane3DReadOnly, EuclidFrameGeometry
{
   @Override
   FramePoint3DReadOnly getPoint();

   @Override
   FrameUnitVector3DReadOnly getNormal();

   /**
    * Computes the minimum distance to the given 3D point and this plane. Frame of the point must be in this {@link #getReferenceFrame()}. If not, throws
    * {@link ReferenceFrameMismatchException}.
    *
    * @param point - 3D point to compute the distance from the plane. Not modified.
    * @return the minimum distance between the 3D point and this 3D plane.
    */
   default double distance(FramePoint3DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return distance((Point3DReadOnly) point);
   }

   /**
    * Computes the minimum signed distance to the given 3D point and this plane. Frame of the point must be in this {@link #getReferenceFrame()}. If not, throws
    * {@link ReferenceFrameMismatchException}. The returned value is negative when the query is located below thep lane, positive otherwise.
    *
    * @param point - 3D point to compute the distance from the plane. Not modified.
    * @return the minimum distance between the 3D point and this 3D plane.
    */
   default double signedDistance(FramePoint3DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return signedDistance((Point3DReadOnly) point);
   }

   /**
    * Computes the coordinates of the intersection between this plane and an infinitely long 3D line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the line is parallel to the plane, this methods fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param line               the line that may intersect this plane. Not modified.
    * @param intersectionToPack point in which the coordinates of the intersection are stored.
    * @return {@code true} if the method succeeds, {@code false} otherwise.
    */
   default boolean intersectionWith(FrameLine3DReadOnly line, FramePoint3DBasics intersectionToPack)
   {
      checkReferenceFrameMatch(line, intersectionToPack);
      return intersectionWith(intersectionToPack, line.getPoint(), line.getDirection());
   }

   /**
    * Computes the coordinates of the intersection between this plane and an infinitely long 3D line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the line is parallel to the plane, this methods fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointOnLine        a point located on the line. Not modified.
    * @param lineDirection      the direction of the line. Not modified.
    * @param intersectionToPack point in which the coordinates of the intersection are stored.
    * @return {@code true} if the method succeeds, {@code false} otherwise.
    */
   default boolean intersectionWith(FramePoint3DBasics intersectionToPack, FramePoint3DReadOnly pointOnLine, FrameVector3DReadOnly lineDirection)
   {
      checkReferenceFrameMatch(intersectionToPack, pointOnLine, lineDirection);
      return EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(getPoint(), getNormal(), pointOnLine, lineDirection, intersectionToPack);
   }

   /**
    * Tests if this plane and the given plane are coincident:
    * <ul>
    * <li>{@code this.normal} and {@code otherPlane.normal} are collinear given the tolerance
    * {@code angleEpsilon}.
    * <li>the distance of {@code otherPlane.point} from the this plane is less than
    * {@code distanceEpsilon}.
    * </ul>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of either normal is below {@code 1.0E-7}, this method fails and returns
    * {@code false}.
    * </ul>
    * </p>
    *
    * @param otherPlane      the other plane to do the test with. Not modified.
    * @param angleEpsilon    tolerance on the angle in radians to determine if the plane normals are
    *                        collinear.
    * @param distanceEpsilon tolerance on the distance to determine if {@code otherPlane.point} belongs
    *                        to this plane.
    * @return {@code true} if the two planes are coincident, {@code false} otherwise.
    * @throws IllegalArgumentException if <tt>angleEpsilon</tt> &notin; [0; <i>pi</i>/2]
    */
   default boolean isCoincident(FramePlane3DReadOnly otherPlane, double angleEpsilon, double distanceEpsilon)
   {
      checkReferenceFrameMatch(otherPlane);
      return EuclidGeometryTools.arePlane3DsCoincident(getPoint(), getNormal(), otherPlane.getPoint(), otherPlane.getNormal(), angleEpsilon, distanceEpsilon);
   }

   /**
    * Tests if the query point is located strictly on or above this plane.
    * <p>
    * Above is defined as the side of the plane toward which the normal is pointing.
    * </p>
    *
    * @param pointToTest the coordinates of the query. Not modified.
    * @return {@code true} if the query is strictly on or above this plane, {@code false} otherwise.
    */
   default boolean isOnOrAbove(FramePoint3DReadOnly pointToTest)
   {
      return isOnOrAbove(pointToTest, 0.0);
   }

   /**
    * Tests if the query point is located on or above this plane given the tolerance {@code epsilon}.
    * <p>
    * Above is defined as the side of the plane toward which the normal is pointing.
    * </p>
    * <p>
    * <ol>
    * <li>if {@code epsilon == 0}, the query has to be either exactly on the plane or strictly above
    * for this method to return {@code true}.
    * <li>if {@code epsilon > 0}, this method returns {@code true} if the query meets the requirements
    * of 1., in addition, this method returns also {@code true} if the query is below the plane at a
    * distance less or equal than {@code epsilon}.
    * <li>if {@code epsilon < 0}, this method returns {@code true} only if the query is above the plane
    * and at a distance of at least {@code Math.abs(epsilon)}.
    * </ol>
    * </p>
    *
    * @param pointToTest the coordinates of the query. Not modified.
    * @param epsilon     the tolerance to use for the test.
    * @return {@code true} if the query is considered to be on or above this plane, {@code false}
    *         otherwise.
    */
   default boolean isOnOrAbove(FramePoint3DReadOnly pointToTest, double epsilon)
   {
      checkReferenceFrameMatch(pointToTest);
      return isOnOrAbove(pointToTest.getX(), pointToTest.getY(), pointToTest.getZ(), epsilon);
   }

   /**
    * Tests if the query point is located strictly on or below this plane.
    * <p>
    * Below is defined as the side of the plane which the normal is pointing away from.
    * </p>
    *
    * @param pointToTest the coordinates of the query. Not modified.
    * @return {@code true} if the query is strictly on or below this plane, {@code false} otherwise.
    */
   default boolean isOnOrBelow(FramePoint3DReadOnly pointToTest)
   {
      return isOnOrBelow(pointToTest, 0.0);
   }

   /**
    * Tests if the query point is located on or below this plane given the tolerance {@code epsilon}.
    * <p>
    * Below is defined as the side of the plane which the normal is pointing away from.
    * </p>
    * <p>
    * <ol>
    * <li>if {@code epsilon == 0}, the query has to be either exactly on the plane or strictly below
    * for this method to return {@code true}.
    * <li>if {@code epsilon > 0}, this method returns {@code true} if the query meets the requirements
    * of 1., in addition, this method returns also {@code true} if the query is above the plane at a
    * distance less or equal than {@code epsilon}.
    * <li>if {@code epsilon < 0}, this method returns {@code true} only if the query is below the plane
    * and at a distance of at least {@code Math.abs(epsilon)}.
    * </ol>
    * </p>
    *
    * @param pointToTest the coordinates of the query. Not modified.
    * @param epsilon     the tolerance to use for the test.
    * @return {@code true} if the query is considered to be on or below this plane, {@code false}
    *         otherwise.
    */
   default boolean isOnOrBelow(FramePoint3DReadOnly pointToTest, double epsilon)
   {
      checkReferenceFrameMatch(pointToTest);
      return isOnOrBelow(pointToTest.getX(), pointToTest.getY(), pointToTest.getZ(), epsilon);
   }


   /**
    * Tests if the two planes are parallel by testing if their normals are collinear. The latter is
    * done given a tolerance on the angle between the two normal axes in the range ]0; <i>pi</i>/2[.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of either normal is below {@code 1.0E-7}, this method fails and returns
    * {@code false}.
    * </ul>
    * </p>
    *
    * @param otherPlane   the other plane to do the test with. Not modified.
    * @param angleEpsilon tolerance on the angle in radians.
    * @return {@code true} if the two planes are parallel, {@code false} otherwise.
    * @throws IllegalArgumentException if <tt>angleEpsilon</tt> &notin; [0; <i>pi</i>/2]
    */
   default boolean isParallel(FramePlane3DReadOnly otherPlane, double angleEpsilon)
   {
      checkReferenceFrameMatch(otherPlane);
      return EuclidGeometryTools.areVector3DsParallel(getNormal(), otherPlane.getNormal(), angleEpsilon);
   }

   /**
    * Computes the orthogonal projection of the given 3D point on this 3D plane.
    *
    * @param pointToProject the point to project on this plane. Modified.
    * @return whether the method succeeded or not.
    */
   default boolean orthogonalProjection(FramePoint3DBasics pointToProject)
   {
      checkReferenceFrameMatch(pointToProject);
      return orthogonalProjection(pointToProject, pointToProject);
   }

   /**
    * Computes the orthogonal projection of the given 3D point on this 3D plane.
    *
    * @param pointToProject   the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto the plane is stored.
    *                         Modified.
    * @return whether the method succeeded or not.
    */
   default boolean orthogonalProjection(FramePoint3DReadOnly pointToProject, FramePoint3DBasics projectionToPack)
   {
      checkReferenceFrameMatch(pointToProject, projectionToPack);
      return EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointToProject, getPoint(), getNormal(), projectionToPack);
   }


   /**
    * {@inheritDoc}
    * <p>
    * This method will return {@code false} if the two planes are physically the same but either the
    * point or vector of each plane is different. For instance, if {@code this.point == other.point}
    * and {@code this.normal == - other.normal}, the two planes are physically the same but this method
    * returns {@code false}.
    * </p>
    */
   @Override
   default boolean epsilonEquals(EuclidFrameGeometry geometry, double epsilon)
   {
      Plane3DReadOnly.super.epsilonEquals(geometry, epsilon);
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof FramePlane3DReadOnly other))
         return false;
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
      return other.getNormal().epsilonEquals(getNormal(), epsilon) && other.getPoint().epsilonEquals(getPoint(), epsilon);
   }

   /** {@inheritDoc} */
   @Override
   default boolean equals(EuclidGeometry geometry)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof FramePlane3DReadOnly other))
         return false;
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
      return getPoint().equals(other.getPoint()) && getNormal().equals(other.getNormal());
   }

   /** {@inheritDoc} */
   @Override
   default boolean geometricallyEquals(EuclidGeometry geometry, double epsilon)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof FramePlane3DReadOnly other))
         return false;
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
      return isCoincident(other, epsilon, epsilon);
   }

}
