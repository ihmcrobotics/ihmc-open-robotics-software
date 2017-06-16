package us.ihmc.robotics.geometry.shapes;

import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Represents an infinitely wide and long 3D plane defined by a 3D point and a 3D unit-vector.
 */
public class Plane3d implements GeometryObject<Plane3d>
{
   private final static double minAllowableVectorPart = Math.sqrt(Double.MIN_NORMAL);

   /** Coordinates of a point located on this plane. */
   private final Point3D point = new Point3D();
   /**
    * Normal of this plane of unit-length. Its direction indicates which side of the plane is
    * considered to be the 'above' part.
    */
   private final Vector3D normal = new Vector3D();

   private boolean hasPointBeenSet = false;
   private boolean hasNormalBeenSet = false;

   /**
    * Default constructor that initializes both {@link #point} and {@link #normal} to zero. This
    * point and vector have to be set to valid values to make this plane usable.
    */
   public Plane3d()
   {
      hasPointBeenSet = false;
      hasNormalBeenSet = false;
   }

   /**
    * Creates a new plane 3D and initializes it to {@code other}.
    * 
    * @param other the other plane used to initialize this plane. Not modified.
    * @throws RuntimeException if the other plane has not been initialized yet.
    */
   public Plane3d(Plane3d other)
   {
      set(other);
   }

   /**
    * Initializes this plane to be passing through the three given points.
    * 
    * @param firstPointOnLine first point on this plane. Not modified.
    * @param secondPointOnLine second point on this plane. Not modified.
    * @param thirdPointOnLine second point on this plane. Not modified.
    * @throws RuntimeException if at least two of the given points are exactly equal.
    * @throws RuntimeException if the plane normal could not be computed from the three given
    *            points.
    */
   public Plane3d(Point3DReadOnly firstPointOnPlane, Point3DReadOnly secondPointOnPlane, Point3DReadOnly thirdPointOnPlane)
   {
      set(firstPointOnPlane, secondPointOnPlane, thirdPointOnPlane);
   }

   /**
    * Initializes this plane to be passing through the given point, with the vector as the normal.
    * 
    * @param pointOnPlane point on this plane. Not modified.
    * @param planeNormal normal of this plane. Not modified.
    * @throws RuntimeException if the new normal is unreasonably small.
    */
   public Plane3d(Point3DReadOnly pointOnPlane, Vector3DReadOnly planeNormal)
   {
      set(pointOnPlane, planeNormal);
   }

   /**
    * Transforms this plane using the given homogeneous transformation matrix.
    * 
    * @param transform the transform to apply on this plane's point and normal. Not modified.
    * @throws RuntimeException if this plane has not been initialized yet.
    */
   @Override
   public void applyTransform(Transform transform)
   {
      checkHasBeenInitialized();
      point.applyTransform(transform);
      normal.applyTransform(transform);
   }

   /**
    * Transforms this plane using the inverse of the given homogeneous transformation matrix.
    * 
    * @param transform the transform to apply on this plane's point and normal. Not modified.
    * @throws RuntimeException if this plane has not been initialized yet.
    */
   @Override
   public void applyInverseTransform(Transform transform)
   {
      checkHasBeenInitialized();
      point.applyInverseTransform(transform);
      normal.applyInverseTransform(transform);
   }

   private void checkDistinctPoints(Point3DReadOnly firstPointOnPlane, Point3DReadOnly secondPointOnPlane, Point3DReadOnly thirdPointOnPlane)
   {
      if (firstPointOnPlane.equals(secondPointOnPlane) || firstPointOnPlane.equals(thirdPointOnPlane) || secondPointOnPlane.equals(thirdPointOnPlane))
      {
         throw new RuntimeException("Tried to create a plane with at least two coincidal points.");
      }
   }

   private void checkHasBeenInitialized()
   {
      if (!hasPointBeenSet)
         throw new RuntimeException("The point of this plane has not been initialized.");
      if (!hasNormalBeenSet)
         throw new RuntimeException("The normal of this plane has not been initialized.");
   }

   private void checkReasonableVector(Vector3DReadOnly localVector)
   {
      if (Math.abs(localVector.getX()) < minAllowableVectorPart && Math.abs(localVector.getY()) < minAllowableVectorPart
            && Math.abs(localVector.getZ()) < minAllowableVectorPart)
      {
         throw new RuntimeException("Plane's normal length must be greater than zero.");
      }
   }

   /**
    * Tests if this plane contains {@link Double#NaN}.
    * 
    * @return {@code true} if {@link #point} and/or {@link #normal} contains {@link Double#NaN},
    *         {@code false} otherwise.
    */
   @Override
   public boolean containsNaN()
   {
      return point.containsNaN() || normal.containsNaN();
   }

   /**
    * Computes the minimum distance the given 3D point and this plane.
    *
    * @param point 3D point to compute the distance from the plane. Not modified.
    * @return the minimum distance between the 3D point and this 3D plane.
    * @throws RuntimeException if this plane has not been initialized yet.
    */
   public double distance(Point3DReadOnly point)
   {
      checkHasBeenInitialized();
      return EuclidGeometryTools.distanceFromPoint3DToPlane3D(point, this.point, normal);
   }

   /**
    * Tests on a per-component basis on the point and normal if this plane is equal to {@code other}
    * with the tolerance {@code epsilon}. This method will return {@code false} if the two planes
    * are physically the same but either the point or vector of each plane is different. For
    * instance, if {@code this.point == other.point} and {@code this.normal == - other.normal}, the
    * two planes are physically the same but this method returns {@code false}.
    * 
    * @param other the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two planes are equal, {@code false} otherwise.
    * @throws RuntimeException if this plane has not been initialized yet.
    */
   @Override
   public boolean epsilonEquals(Plane3d other, double epsilon)
   {
      checkHasBeenInitialized();
      return other.normal.epsilonEquals(normal, epsilon) && other.point.epsilonEquals(point, epsilon);
   }

   /**
    * Tests on a per component basis, if this plane 3D is exactly equal to {@code other}.
    *
    * @param other the other plane 3D to compare against this. Not modified.
    * @return {@code true} if the two planes are exactly equal component-wise, {@code false}
    *         otherwise.
    */
   public boolean equals(Plane3d other)
   {
      if (other == null)
         return false;
      else
         return point.equals(other.point) && normal.equals(other.normal);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method
    * returns {@link #equals(Plane3D)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object obj)
   {
      try
      {
         return equals((Plane3d) obj);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   /**
    * Gets the read-only reference to the normal of this plane.
    * 
    * @return the reference to the normal.
    * @throws RuntimeException if this plane has not been initialized yet.
    */
   public Vector3DReadOnly getNormal()
   {
      checkHasBeenInitialized();
      return normal;
   }

   /**
    * Gets the direction defining this plane by storing its components in the given argument
    * {@code planeNormalToPack}.
    * 
    * @param planeNormalToPack vector in which the components of this plane's normal are stored.
    *           Modified.
    * @throws RuntimeException if this plane has not been initialized yet.
    */
   public void getNormal(Vector3DBasics planeNormalToPack)
   {
      checkHasBeenInitialized();
      planeNormalToPack.set(normal);
   }

   /**
    * Returns a copy of this plane's normal.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @return a copy of this plane's normal.
    * @throws RuntimeException if this plane has not been initialized yet.
    */
   public Vector3D getNormalCopy()
   {
      checkHasBeenInitialized();
      return new Vector3D(normal);
   }

   /**
    * Gets the x-component of this plane's normal.
    * 
    * @return the x-component of this plane's normal.
    * @throws RuntimeException if this plane has not been initialized yet.
    */
   public double getNormalX()
   {
      checkHasBeenInitialized();
      return normal.getX();
   }

   /**
    * Gets the y-component of this plane's normal.
    * 
    * @return the y-component of this plane's normal.
    * @throws RuntimeException if this plane has not been initialized yet.
    */
   public double getNormalY()
   {
      checkHasBeenInitialized();
      return normal.getY();
   }

   /**
    * Gets the z-component of this plane's normal.
    * 
    * @return the z-component of this plane's normal.
    * @throws RuntimeException if this plane has not been initialized yet.
    */
   public double getNormalZ()
   {
      checkHasBeenInitialized();
      return normal.getZ();
   }

   /**
    * Gets the read-only reference to the point through which this plane is going.
    * 
    * @return the reference to the point.
    * @throws RuntimeException if this plane has not been initialized yet.
    */
   public Point3DReadOnly getPoint()
   {
      checkHasBeenInitialized();
      return point;
   }

   /**
    * Gets the point defining this plane by storing its coordinates in the given argument
    * {@code pointToPack}.
    * 
    * @param pointToPack point in which the coordinates of this plane's point are stored. Modified.
    * @throws RuntimeException if this plane has not been initialized yet.
    */
   public void getPoint(Point3DBasics pointOnPlaneToPack)
   {
      checkHasBeenInitialized();
      pointOnPlaneToPack.set(point);
   }

   /**
    * Returns a copy of the point defining this plane.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @return a copy of this plane's point.
    * @throws RuntimeException if this plane has not been initialized yet.
    */
   public Point3D getPointCopy()
   {
      checkHasBeenInitialized();
      return new Point3D(point);
   }

   /**
    * Gets the x-coordinate of a point this plane goes through.
    * 
    * @return the x-coordinate of this plane's point.
    * @throws RuntimeException if this plane has not been initialized yet.
    */
   public double getPointX()
   {
      checkHasBeenInitialized();
      return point.getX();
   }

   /**
    * Gets the y-coordinate of a point this plane goes through.
    * 
    * @return the y-coordinate of this plane's point.
    * @throws RuntimeException if this plane has not been initialized yet.
    */
   public double getPointY()
   {
      checkHasBeenInitialized();
      return point.getY();
   }

   /**
    * Gets the z-coordinate of a point this plane goes through.
    * 
    * @return the z-coordinate of this plane's point.
    * @throws RuntimeException if this plane has not been initialized yet.
    */
   public double getPointZ()
   {
      checkHasBeenInitialized();
      return point.getZ();
   }

   /**
    * Computes the z-coordinate such that the point at (x, y, z) is located on this plane.
    * 
    * @param x the x-coordinate of the query.
    * @param y the y-coordinate of the query.
    * @return the z-coordinate of the plane.
    * @throws RuntimeException if this plane has not been initialized yet.
    */
   public double getZOnPlane(double x, double y)
   {
      checkHasBeenInitialized();

      // The three components of the plane origin
      double x0 = point.getX();
      double y0 = point.getY();
      double z0 = point.getZ();
      // The three components of the plane normal
      double a = normal.getX();
      double b = normal.getY();
      double c = normal.getZ();

      // Given the plane equation: a*x + b*y + c*z + d = 0, with d = -(a*x0 + b*y0 + c*z0), we find z:
      double z = a / c * (x0 - x) + b / c * (y0 - y) + z0;
      return z;
   }

   /**
    * Computes the coordinates of the intersection between this plane and an infinitely long 3D
    * line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the line is parallel to the plane, this methods fails and returns {@code false}.
    * </ul>
    * </p>
    * 
    * @param line the line that may intersect this plane. Not modified.
    * @param intersectionToPack point in which the coordinates of the intersection are stored.
    * @return {@code true} if the method succeeds, {@code false} otherwise.
    * @throws RuntimeException if this plane has not been initialized yet.
    */
   public boolean intersectionWith(Line3D line, Point3DBasics intersectionToPack)
   {
      checkHasBeenInitialized();
      return intersectionWith(intersectionToPack, line.getPoint(), line.getDirection());
   }

   /**
    * Computes the coordinates of the intersection between this plane and an infinitely long 3D
    * line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the line is parallel to the plane, this methods fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointOnLine a point located on the line. Not modified.
    * @param lineDirection the direction of the line. Not modified.
    * @param intersectionToPack point in which the coordinates of the intersection are stored.
    * @return {@code true} if the method succeeds, {@code false} otherwise.
    * @throws RuntimeException if this plane has not been initialized yet.
    */
   public boolean intersectionWith(Point3DBasics intersectionToPack, Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection)
   {
      checkHasBeenInitialized();
      return EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(point, normal, pointOnLine, lineDirection, intersectionToPack);
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
    * @param otherPlane the other plane to do the test with. Not modified.
    * @param angleEpsilon tolerance on the angle in radians to determine if the plane normals are
    *           collinear.
    * @param distanceEpsilon tolerance on the distance to determine if {@code otherPlane.point}
    *           belongs to this plane.
    * @return {@code true} if the two planes are coincident, {@code false} otherwise.
    * @throws RuntimeException if this plane has not been initialized yet.
    */
   public boolean isCoincident(Plane3d otherPlane, double angleEpsilon, double distanceEpsilon)
   {
      checkHasBeenInitialized();
      return EuclidGeometryTools.arePlane3DsCoincident(point, normal, otherPlane.point, otherPlane.normal, angleEpsilon, distanceEpsilon);
   }

   /**
    * Tests if the query point is located on or above this plane given the tolerance
    * {@code epsilon}.
    * <p>
    * Above is defined as the side of the plane toward which the normal is pointing.
    * </p>
    * <p>
    * <ol>
    * <li>if {@code epsilon == 0}, the query has to be either exactly on the plane or strictly above
    * for this method to return {@code true}.
    * <li>if {@code epsilon > 0}, this method returns {@code true} if the query meets the
    * requirements of 1., in addition, this method returns also {@code true} if the query is below
    * the plane at a distance less or equal than {@code epsilon}.
    * <li>if {@code epsilon < 0}, this method returns {@code true} only if the query is above the
    * plane and at a distance of at least {@code Math.abs(epsilon)}.
    * </ol>
    * </p>
    * 
    * @param x the x-coordinate of the query.
    * @param y the y-coordinate of the query.
    * @param z the z-coordinate of the query.
    * @param epsilon the tolerance to use for the test.
    * @return {@code true} if the query is considered to be on or above this plane, {@code false}
    *         otherwise.
    * @throws RuntimeException if this plane has not been initialized yet.
    */
   public boolean isOnOrAbove(double x, double y, double z, double epsilon)
   {
      checkHasBeenInitialized();
      double dx = (x - point.getX()) * normal.getX();
      double dy = (y - point.getY()) * normal.getY();
      double dz = (z - point.getZ()) * normal.getZ();

      return (dx + dy + dz) >= -epsilon;
   }

   /**
    * Tests if the query point is located strictly on or above this plane.
    * <p>
    * Above is defined as the side of the plane toward which the normal is pointing.
    * </p>
    * 
    * @param pointToTest the coordinates of the query. Not modified
    * @return {@code true} if the query is strictly on or above this plane, {@code false} otherwise.
    * @throws RuntimeException if this plane has not been initialized yet.
    */
   public boolean isOnOrAbove(Point3DReadOnly pointToTest)
   {
      return isOnOrAbove(pointToTest, 0.0);
   }

   /**
    * Tests if the query point is located on or above this plane given the tolerance
    * {@code epsilon}.
    * <p>
    * Above is defined as the side of the plane toward which the normal is pointing.
    * </p>
    * <p>
    * <ol>
    * <li>if {@code epsilon == 0}, the query has to be either exactly on the plane or strictly above
    * for this method to return {@code true}.
    * <li>if {@code epsilon > 0}, this method returns {@code true} if the query meets the
    * requirements of 1., in addition, this method returns also {@code true} if the query is below
    * the plane at a distance less or equal than {@code epsilon}.
    * <li>if {@code epsilon < 0}, this method returns {@code true} only if the query is above the
    * plane and at a distance of at least {@code Math.abs(epsilon)}.
    * </ol>
    * </p>
    * 
    * @param pointToTest the coordinates of the query. Not modified
    * @param epsilon the tolerance to use for the test.
    * @return {@code true} if the query is considered to be on or above this plane, {@code false}
    *         otherwise.
    * @throws RuntimeException if this plane has not been initialized yet.
    */
   public boolean isOnOrAbove(Point3DReadOnly pointToTest, double epsilon)
   {
      return isOnOrAbove(pointToTest.getX(), pointToTest.getY(), pointToTest.getZ(), epsilon);
   }

   /**
    * Tests if the query point is located on or below this plane given the tolerance
    * {@code epsilon}.
    * <p>
    * Below is defined as the side of the plane which the normal is pointing away from.
    * </p>
    * <p>
    * <ol>
    * <li>if {@code epsilon == 0}, the query has to be either exactly on the plane or strictly below
    * for this method to return {@code true}.
    * <li>if {@code epsilon > 0}, this method returns {@code true} if the query meets the
    * requirements of 1., in addition, this method returns also {@code true} if the query is above
    * the plane at a distance less or equal than {@code epsilon}.
    * <li>if {@code epsilon < 0}, this method returns {@code true} only if the query is below the
    * plane and at a distance of at least {@code Math.abs(epsilon)}.
    * </ol>
    * </p>
    * 
    * @param x the x-coordinate of the query.
    * @param y the y-coordinate of the query.
    * @param z the z-coordinate of the query.
    * @param epsilon the tolerance to use for the test.
    * @return {@code true} if the query is considered to be on or below this plane, {@code false}
    *         otherwise.
    * @throws RuntimeException if this plane has not been initialized yet.
    */
   public boolean isOnOrBelow(double x, double y, double z, double epsilon)
   {
      checkHasBeenInitialized();
      double dx = (x - point.getX()) * normal.getX();
      double dy = (y - point.getY()) * normal.getY();
      double dz = (z - point.getZ()) * normal.getZ();

      return (dx + dy + dz) <= epsilon;
   }

   /**
    * Tests if the query point is located strictly on or below this plane.
    * <p>
    * Below is defined as the side of the plane which the normal is pointing away from.
    * </p>
    * 
    * @param pointToTest the coordinates of the query. Not modified
    * @return {@code true} if the query is strictly on or below this plane, {@code false} otherwise.
    * @throws RuntimeException if this plane has not been initialized yet.
    */
   public boolean isOnOrBelow(Point3DReadOnly pointToTest)
   {
      return isOnOrBelow(pointToTest, 0.0);
   }

   /**
    * Tests if the query point is located on or below this plane given the tolerance
    * {@code epsilon}.
    * <p>
    * Below is defined as the side of the plane which the normal is pointing away from.
    * </p>
    * <p>
    * <ol>
    * <li>if {@code epsilon == 0}, the query has to be either exactly on the plane or strictly below
    * for this method to return {@code true}.
    * <li>if {@code epsilon > 0}, this method returns {@code true} if the query meets the
    * requirements of 1., in addition, this method returns also {@code true} if the query is above
    * the plane at a distance less or equal than {@code epsilon}.
    * <li>if {@code epsilon < 0}, this method returns {@code true} only if the query is below the
    * plane and at a distance of at least {@code Math.abs(epsilon)}.
    * </ol>
    * </p>
    * 
    * @param pointToTest the coordinates of the query. Not modified
    * @param epsilon the tolerance to use for the test.
    * @return {@code true} if the query is considered to be on or below this plane, {@code false}
    *         otherwise.
    * @throws RuntimeException if this plane has not been initialized yet.
    */
   public boolean isOnOrBelow(Point3DReadOnly pointToTest, double epsilon)
   {
      return isOnOrBelow(pointToTest.getX(), pointToTest.getY(), pointToTest.getZ(), epsilon);
   }

   /**
    * Tests if the two planes are parallel by testing if their normals are collinear. The latter is
    * done given a tolerance on the angle between the two normal axes in the range ]0; <i>pi</i>/2[.
    *
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of either normal is below {@code 1.0E-7}, this method fails and returns
    * {@code false}.
    * </ul>
    * </p>
    *
    * @param otherPlane the other plane to do the test with. Not modified.
    * @param angleEpsilon tolerance on the angle in radians.
    * @return {@code true} if the two planes are parallel, {@code false} otherwise.
    * @throws RuntimeException if this plane has not been initialized yet.
    */
   public boolean isParallel(Plane3d otherPlane, double angleEpsilon)
   {
      checkHasBeenInitialized();
      return EuclidGeometryTools.areVector3DsParallel(normal, otherPlane.normal, angleEpsilon);
   }

   /**
    * Computes the orthogonal projection of the given 3D point on this 3D plane.
    *
    * @param pointToProject the point to project on this plane. Modified.
    * @return whether the method succeeded or not.
    * @throws RuntimeException if this plane has not been initialized yet.
    */
   public boolean orthogonalProjection(Point3DBasics pointToProject)
   {
      checkHasBeenInitialized();
      return orthogonalProjection(pointToProject, pointToProject);
   }

   /**
    * Computes the orthogonal projection of the given 3D point on this 3D plane.
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto the plane is stored.
    *           Modified.
    * @throws RuntimeException if this plane has not been initialized yet.
    */
   public boolean orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      checkHasBeenInitialized();
      return EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointToProject, point, normal, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of the given 3D point on this 3D line.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @return the projection of the point onto the plane or {@code null} if the method failed.
    * @throws RuntimeException if this plane has not been initialized yet.
    */
   public Point3D orthogonalProjectionCopy(Point3DReadOnly pointToProject)
   {
      checkHasBeenInitialized();
      return EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointToProject, point, normal);
   }

   /**
    * Sets this plane to be the same as the given plane.
    * 
    * @param other the other plane to copy. Not modified.
    * @throws RuntimeException if the other plane has not been initialized yet.
    */
   @Override
   public void set(Plane3d other)
   {
      point.set(other.getPoint());
      normal.set(other.getNormal());
      hasPointBeenSet = true;
      hasNormalBeenSet = true;
   }

   /**
    * Redefines this plane such that it goes through the three given points.
    * 
    * @param firstPointOnLine first point on this plane. Not modified.
    * @param secondPointOnLine second point on this plane. Not modified.
    * @param thirdPointOnLine second point on this plane. Not modified.
    * @throws RuntimeException if at least two of the given points are exactly equal.
    * @throws RuntimeException if the plane normal could not be computed from the three given
    *            points.
    */
   public void set(Point3DReadOnly firstPointOnPlane, Point3DReadOnly secondPointOnPlane, Point3DReadOnly thirdPointOnPlane)
   {
      checkDistinctPoints(firstPointOnPlane, secondPointOnPlane, thirdPointOnPlane);

      setPoint(firstPointOnPlane);
      boolean success = EuclidGeometryTools.normal3DFromThreePoint3Ds(firstPointOnPlane, secondPointOnPlane, thirdPointOnPlane, normal);

      if (!success)
         throw new RuntimeException("Failed to compute the plane normal. Given points: " + firstPointOnPlane + ", " + secondPointOnPlane + ", "
               + thirdPointOnPlane);

      hasNormalBeenSet = true;
   }

   /**
    * Redefines this plane with a new point and a new normal.
    * 
    * @param pointOnPlane new point on this plane. Not modified.
    * @param planeNormal new normal of this plane. Not modified.
    * @throws RuntimeException if the new normal is unreasonably small.
    */
   public void set(Point3DReadOnly pointOnPlane, Vector3DReadOnly planeNormal)
   {
      setPoint(pointOnPlane);
      setNormal(planeNormal);
   }

   /**
    * Changes the normal of this plane by setting it to the normalized value of the given vector.
    * 
    * @param normalX the new x-component of the normal of this normal.
    * @param normalY the new y-component of the normal of this normal.
    * @param normalZ the new z-component of the normal of this normal.
    * @throws RuntimeException if the new normal is unreasonably small.
    */
   public void setNormal(double normalX, double normalY, double normalZ)
   {
      normal.set(normalX, normalY, normalZ);
      checkReasonableVector(normal);
      normal.normalize();
      hasNormalBeenSet = true;
   }

   /**
    * Changes the direction of this plane by setting it to the normalized value of the given vector.
    * 
    * @param planeNormal new normal of this plane. Not modified.
    * @throws RuntimeException if the new normal is unreasonably small.
    */
   public void setNormal(Vector3DReadOnly planeNormal)
   {
      setNormal(planeNormal.getX(), planeNormal.getY(), planeNormal.getZ());
   }

   /**
    * Changes the point through which this plane has to go.
    * 
    * @param pointX the new x-coordinate of the point on this plane.
    * @param pointY the new y-coordinate of the point on this plane.
    * @param pointZ the new z-coordinate of the point on this plane.
    */
   public void setPoint(double pointX, double pointY, double pointZ)
   {
      point.set(pointX, pointY, pointZ);
      hasPointBeenSet = true;
   }

   /**
    * Changes the point through which this plane has to go.
    * 
    * @param pointOnLine new point on this plane. Not modified.
    */
   public void setPoint(Point3DReadOnly pointOnPlane)
   {
      setPoint(pointOnPlane.getX(), pointOnPlane.getY(), pointOnPlane.getZ());
   }

   /**
    * Sets the point and normal of this plane to {@link Double#NaN}. After calling this method, this
    * plane becomes invalid. A new valid point and valid normal will have to be set so this plane is
    * again usable.
    */
   @Override
   public void setToNaN()
   {
      point.setToNaN();
      normal.setToNaN();
   }

   /**
    * Sets the point and vector of this plane to zero. After calling this method, this plane becomes
    * invalid. A new valid point and valid normal will have to be set so this plane is again usable.
    */
   @Override
   public void setToZero()
   {
      point.setToZero();
      normal.setToZero();
      hasPointBeenSet = false;
      hasNormalBeenSet = false;
   }

   /**
    * Computes the minimum signed distance the given 3D point and this plane.
    * <p>
    * The returned value is negative when the query is located below the plane, positive otherwise.
    * </p>
    *
    * @param point 3D point to compute the distance from the plane. Not modified.
    * @return the signed distance between the point and this plane.
    * @throws RuntimeException if this plane has not been initialized yet.
    */
   public double signedDistance(Point3DReadOnly point)
   {
      checkHasBeenInitialized();
      return EuclidGeometryTools.signedDistanceFromPoint3DToPlane3D(point, this.point, normal);
   }

   /**
    * Provides a {@code String} representation of this plane 3D as follows:<br>
    * Plane 3D: point = (x, y, z), normal = (x, y, z)
    *
    * @return the {@code String} representing this plane 3D.
    */
   @Override
   public String toString()
   {
      return "Plane 3D: point on plane = " + point + ", normal = " + normal;
   }
}
