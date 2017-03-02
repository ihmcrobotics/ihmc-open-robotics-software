package us.ihmc.robotics.geometry.shapes;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.geometry.GeometryTools;

public class Plane3d implements GeometryObject<Plane3d>
{
   private Point3D point = new Point3D();
   private Vector3D normal = new Vector3D(0.0, 0.0, 1.0);
   private Vector3D temporaryVector = new Vector3D();

   public Plane3d()
   {
   }

   public Plane3d(Point3DReadOnly point, Vector3DReadOnly normal)
   {
      this.point.set(point);
      this.normal.set(normal);
      this.normal.normalize();
   }
   
   public Plane3d(Point3DReadOnly pointA, Point3DReadOnly pointB, Point3DReadOnly pointC)
   {
      point.set(pointA);
      double v1_x = pointB.getX() - pointA.getX();
      double v1_y = pointB.getY() - pointA.getY();
      double v1_z = pointB.getZ() - pointA.getZ();
      
      double v2_x = pointC.getX() - pointA.getX();
      double v2_y = pointC.getY() - pointA.getY();
      double v2_z = pointC.getZ() - pointA.getZ();

      double x = v1_y*v2_z - v1_z*v2_y;
      double y = v2_x*v1_z - v2_z*v1_x;
      this.normal.setZ(v1_x*v2_y - v1_y*v2_x);
      this.normal.setX(x);
      this.normal.setY(y);
      this.normal.normalize();
   }

   public Plane3d(Plane3d plane)
   {
      this.point.set(plane.point);
      this.normal.set(plane.normal);
   }

   public void getPoint(Point3DBasics pointToPack)
   {
      pointToPack.set(this.point);
   }
   
   public Point3D getPointCopy()
   {
      Point3D pointToReturn = new Point3D();
      this.getPoint(pointToReturn);
      return pointToReturn;
   }
   
   public Point3D getPoint()
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

   public void setPoints(Point3DReadOnly pointA, Point3DReadOnly pointB, Point3DReadOnly pointC)
   {
      point.set(pointA);
      double v1_x = pointB.getX() - pointA.getX();
      double v1_y = pointB.getY() - pointA.getY();
      double v1_z = pointB.getZ() - pointA.getZ();
      
      double v2_x = pointC.getX() - pointA.getX();
      double v2_y = pointC.getY() - pointA.getY();
      double v2_z = pointC.getZ() - pointA.getZ();

      double x = v1_y*v2_z - v1_z*v2_y;
      double y = v2_x*v1_z - v2_z*v1_x;
      this.normal.setZ(v1_x*v2_y - v1_y*v2_x);
      this.normal.setX(x);
      this.normal.setY(y);
      this.normal.normalize();
   }
   
   public void getNormal(Vector3DBasics normalToPack)
   {
      normalToPack.set(normal);
   }
   
   public Vector3D getNormalCopy()
   {
      Vector3D normalToReturn = new Vector3D();
      this.getNormal(normalToReturn);
      return normalToReturn;
   }
   
   public Vector3D getNormal()
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
      this.normal.set(plane3d.normal);
      this.point.set(plane3d.point);
   }

   public void setNormal(Vector3DReadOnly normal)
   {
      this.normal.set(normal);
   }

   @Override
   public boolean epsilonEquals(Plane3d plane, double epsilon)
   {
      return ((plane.normal.epsilonEquals(normal, epsilon)) && (plane.point.epsilonEquals(point, epsilon)));
   }

   public boolean isOnOrAbove(Point3DReadOnly pointToTest)
   {
      return isOnOrAbove(pointToTest, 0.0);
   }

   public boolean isOnOrAbove(Point3DReadOnly pointToTest, double epsilon)
   {
      temporaryVector.set(pointToTest);
      temporaryVector.sub(this.point);

      return (temporaryVector.dot(this.normal) >= -epsilon);  
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
      temporaryVector.set(x, y, z);
      temporaryVector.sub(this.point);

      return (temporaryVector.dot(this.normal) <= epsilon);
   }

   /**
    * Tests if the two planes are parallel by testing if their normals are collinear.
    * The latter is done given a tolerance on the angle between the two normal axes in the range ]0; <i>pi</i>/2[.
    * 
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if the length of either normal is below {@code 1.0E-7}, this method fails and returns {@code false}.
    * </ul>
    * </p>
    * 
    * @param otherPlane the other plane to do the test with. Not modified.
    * @param angleEpsilon tolerance on the angle in radians.
    * @return {@code true} if the two planes are parallel, {@code false} otherwise.
    */
   public boolean isParallel(Plane3d otherPlane, double angleEpsilon)
   {
      return GeometryTools.areVectorsCollinear(normal, otherPlane.normal, angleEpsilon);
   }

   /**
    * Tests if this plane and the given plane are coincident:
    * <ul>
    *    <li> {@code this.normal} and {@code otherPlane.normal} are collinear given the tolerance {@code angleEpsilon}.
    *    <li> the distance of {@code otherPlane.point} from the this plane is less than {@code distanceEpsilon}.
    * </ul>
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if the length of either normal is below {@code 1.0E-7}, this method fails and returns {@code false}.
    * </ul>
    * </p>
    * 
    * @param otherPlane the other plane to do the test with. Not modified.
    * @param angleEpsilon tolerance on the angle in radians to determine if the plane normals are collinear. 
    * @param distanceEpsilon tolerance on the distance to determine if {@code otherPlane.point} belongs to this plane.
    * @return {@code true} if the two planes are coincident, {@code false} otherwise.
    */
   public boolean isCoincident(Plane3d otherPlane, double angleEpsilon, double distanceEpsilon)
   {
      return GeometryTools.arePlanesCoincident(point, normal, otherPlane.point, otherPlane.normal, angleEpsilon, distanceEpsilon);
   }
   
   public Point3D orthogonalProjectionCopy(Point3DReadOnly point)
   {
      Point3D returnPoint = new Point3D(point);
      orthogonalProjection(returnPoint);
      return returnPoint;
   }
   
   // this method was not tested. Use it at your own risk.
   public void orthogonalProjection(Vector3DBasics vectorToProject)
   {
      temporaryVector.set( 0.0, 0.0, 0.0);
      temporaryVector.sub(this.point);
      double distA = temporaryVector.dot(this.normal);
      
      temporaryVector.set( vectorToProject );
      temporaryVector.sub(this.point);
      double distB = temporaryVector.dot(this.normal);
      
      temporaryVector.set(this.normal);
      temporaryVector.scale(distB - distA);
      vectorToProject.sub( temporaryVector );
   }
   
   public void orthogonalProjection(Point3DBasics pointToProject)
   {
      temporaryVector.set(pointToProject);
      temporaryVector.sub(this.point);
      double temporaryDouble = temporaryVector.dot(this.normal);
      temporaryVector.set(this.normal);
      temporaryVector.scale(temporaryDouble);
      temporaryVector.sub(pointToProject);
      temporaryVector.scale(-1.0);
      pointToProject.set(temporaryVector);
   }
   
   public double getZOnPlane(double x, double y)
   {
     double z = (normal.getX() * (point.getX() - x) + normal.getY() * (point.getY() - y) + normal.getZ() * point.getZ())/normal.getZ();
     if (Double.isInfinite(z)) return Double.NaN;
     return z;
   }

   public double distance(Point3DReadOnly point)
   {
      temporaryVector.set(point);
      temporaryVector.sub(this.point);
      double temporaryDouble = temporaryVector.dot(this.normal);

      return Math.abs(temporaryDouble);
   }

   public double signedDistance(Point3DReadOnly point)
   {
      temporaryVector.set(point);
      temporaryVector.sub(this.point);
      double temporaryDouble = temporaryVector.dot(this.normal);

      return temporaryDouble;
   }



   public Plane3d applyTransformCopy(Transform transformation)
   {
      Plane3d returnPlane = new Plane3d(this);
      returnPlane.applyTransform(transformation);

      return returnPlane;
   }
   
   @Override
   public void applyTransform(Transform transform)
   {
      point.applyTransform(transform);
      normal.applyTransform(transform);
   }

   public void getIntersectionWithLine(Point3DBasics intersectionToPack, Point3DReadOnly lineStart, Vector3DReadOnly lineVector)
   {
      // po = line start, p1 = line end
      // v0 = point on plane
      // n = plane normal
      // intersection point is p(s) = p0 + s*(p1 - p0)
      // scalar s = (n dot (v0 - p0))/(n dot (p1 - p0)

      Vector3D fromP0toV0 = new Vector3D(point);
      fromP0toV0.sub(lineStart);

      double scaleFactor = normal.dot(fromP0toV0) / normal.dot(lineVector);
      intersectionToPack.scaleAdd(scaleFactor, lineVector, lineStart);
   }

   @Override
   public boolean containsNaN()
   {
      if (Double.isNaN(point.getX())) return true;
      if (Double.isNaN(point.getY())) return true;
      if (Double.isNaN(point.getZ())) return true;
      
      if (Double.isNaN(normal.getX())) return true;
      if (Double.isNaN(normal.getY())) return true;
      if (Double.isNaN(normal.getZ())) return true;
      
      return false;
   }

   @Override
   public void setToNaN()
   {
      setPoint(Double.NaN, Double.NaN, Double.NaN);
      setNormal(Double.NaN, Double.NaN, Double.NaN);
   }
   
   @Override
   public void setToZero()
   {
   }

   @Override
   public String toString()
   {
      StringBuilder builder = new StringBuilder();
      
      builder.append("point = " + point + ", normal = " + normal + "\n");
      
      return builder.toString();
   }
}
