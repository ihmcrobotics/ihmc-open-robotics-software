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

public class Plane3d implements GeometryObject<Plane3d>
{
   private Point3D point = new Point3D();
   private Vector3D normal = new Vector3D(0.0, 0.0, 1.0);

   public Plane3d()
   {
   }

   public Plane3d(Point3DReadOnly pointOnPlane, Vector3DReadOnly planeNormal)
   {
      set(pointOnPlane, planeNormal);
   }

   public Plane3d(Point3DReadOnly firstPointOnPlane, Point3DReadOnly secondPointOnPlane, Point3DReadOnly thirdPointOnPlane)
   {
      setPoints(firstPointOnPlane, secondPointOnPlane, thirdPointOnPlane);
   }

   public Plane3d(Plane3d other)
   {
      set(other);
   }

   public void getPoint(Point3DBasics pointOnPlaneToPack)
   {
      pointOnPlaneToPack.set(point);
   }

   public Point3D getPointCopy()
   {
      return new Point3D(point);
   }

   public Point3DReadOnly getPoint()
   {
      return point;
   }

   public void setPoint(Point3DReadOnly point)
   {
      this.point.set(point);
   }

   public void setPoint(double x, double y, double z)
   {
      point.set(x, y, z);
   }

   public void setPoints(Point3DReadOnly firstPointOnPlane, Point3DReadOnly secondPointOnPlane, Point3DReadOnly thirdPointOnPlane)
   {
      point.set(firstPointOnPlane);
      boolean success = EuclidGeometryTools.normal3DFromThreePoint3Ds(firstPointOnPlane, secondPointOnPlane, thirdPointOnPlane, normal);

      if (!success)
         throw new RuntimeException("Failed to compute the plane normal. Given points: " + firstPointOnPlane + ", " + secondPointOnPlane + ", "
               + thirdPointOnPlane);
   }

   public void getNormal(Vector3DBasics planeNormalToPack)
   {
      planeNormalToPack.set(normal);
   }

   public Vector3D getNormalCopy()
   {
      return new Vector3D(normal);
   }

   public Vector3DReadOnly getNormal()
   {
      return normal;
   }

   public void setNormal(double x, double y, double z)
   {
      normal.set(x, y, z);
      normal.normalize();
   }

   @Override
   public void set(Plane3d plane3d)
   {
      point.set(plane3d.point);
      normal.set(plane3d.normal);
   }

   public void set(Point3DReadOnly pointOnPlane, Vector3DReadOnly planeNormal)
   {
      point.set(pointOnPlane);
      normal.set(planeNormal);
      normal.normalize();
   }

   public void setNormal(Vector3DReadOnly normal)
   {
      this.normal.set(normal);
   }

   @Override
   public boolean epsilonEquals(Plane3d plane, double epsilon)
   {
      return plane.normal.epsilonEquals(normal, epsilon) && plane.point.epsilonEquals(point, epsilon);
   }

   public boolean isOnOrAbove(Point3DReadOnly pointToTest)
   {
      return isOnOrAbove(pointToTest, 0.0);
   }

   public boolean isOnOrAbove(Point3DReadOnly pointToTest, double epsilon)
   {
      return isOnOrAbove(pointToTest.getX(), pointToTest.getY(), pointToTest.getZ(), epsilon);
   }

   public boolean isOnOrAbove(double x, double y, double z, double epsilon)
   {
      double dx = (x - point.getX()) * normal.getX();
      double dy = (y - point.getY()) * normal.getY();
      double dz = (z - point.getZ()) * normal.getZ();

      return (dx + dy + dz) >= -epsilon;
   }

   public boolean isOnOrBelow(Point3DReadOnly pointToTest)
   {
      return isOnOrBelow(pointToTest, 0.0);
   }

   public boolean isOnOrBelow(Point3DReadOnly pointToTest, double epsilon)
   {
      return isOnOrBelow(pointToTest.getX(), pointToTest.getY(), pointToTest.getZ(), epsilon);
   }

   public boolean isOnOrBelow(double x, double y, double z, double epsilon)
   {
      double dx = (x - point.getX()) * normal.getX();
      double dy = (y - point.getY()) * normal.getY();
      double dz = (z - point.getZ()) * normal.getZ();

      return (dx + dy + dz) <= epsilon;
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
    */
   public boolean isParallel(Plane3d otherPlane, double angleEpsilon)
   {
      return EuclidGeometryTools.areVector3DsParallel(normal, otherPlane.normal, angleEpsilon);
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
    */
   public boolean isCoincident(Plane3d otherPlane, double angleEpsilon, double distanceEpsilon)
   {
      return EuclidGeometryTools.arePlane3DsCoincident(point, normal, otherPlane.point, otherPlane.normal, angleEpsilon, distanceEpsilon);
   }

   public boolean orthogonalProjection(Point3DBasics pointToProject)
   {
      return orthogonalProjection(pointToProject, pointToProject);
   }

   public boolean orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      return EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointToProject, point, normal, projectionToPack);
   }

   public Point3D orthogonalProjectionCopy(Point3DReadOnly pointToProject)
   {
      return EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointToProject, point, normal);
   }

   public double getZOnPlane(double x, double y)
   {
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

   public double distance(Point3DReadOnly point)
   {
      return EuclidGeometryTools.distanceFromPoint3DToPlane3D(point, this.point, normal);
   }

   public double signedDistance(Point3DReadOnly point)
   {
      return EuclidGeometryTools.signedDistanceFromPoint3DToPlane3D(point, this.point, normal);
   }

   @Override
   public void applyTransform(Transform transform)
   {
      point.applyTransform(transform);
      normal.applyTransform(transform);
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      point.applyInverseTransform(transform);
      normal.applyInverseTransform(transform);
   }

   public boolean intersectionWith(Point3DBasics intersectionToPack, Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection)
   {
      return EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(point, normal, pointOnLine, lineDirection, intersectionToPack);
   }

   public boolean intersectionWith(Point3DBasics intersectionToPack, Line3D line)
   {
      return intersectionWith(intersectionToPack, line.getPoint(), line.getDirection());
   }

   @Override
   public boolean containsNaN()
   {
      return point.containsNaN() || normal.containsNaN();
   }

   @Override
   public void setToNaN()
   {
      point.setToNaN();
      normal.setToNaN();
   }

   @Override
   public void setToZero()
   {
      point.setToZero();
      normal.setToZero();
   }

   @Override
   public String toString()
   {
      return "Plane 3D: point on plane: " + point + ", normal: " + normal;
   }
}
