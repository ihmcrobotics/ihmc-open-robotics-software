package us.ihmc.robotics.geometry.shapes;

import us.ihmc.robotics.geometry.RigidBodyTransform;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class Plane3d
{
   private Point3d point = new Point3d();
   private Vector3d normal = new Vector3d(0.0, 0.0, 1.0);
   private Vector3d temporaryVector = new Vector3d();

   public Plane3d()
   {
   }

   public Plane3d(Point3d point, Vector3d normal)
   {
      this.point.set(point);
      this.normal.set(normal);
      this.normal.normalize();
   }
   
   public Plane3d(Point3d pointA, Point3d pointB, Point3d pointC)
   {
      point.set(pointA);
      double v1_x = pointB.x - pointA.x;
      double v1_y = pointB.y - pointA.y;
      double v1_z = pointB.z - pointA.z;
      
      double v2_x = pointC.x - pointA.x;
      double v2_y = pointC.y - pointA.y;
      double v2_z = pointC.z - pointA.z;

      double x = v1_y*v2_z - v1_z*v2_y;
      double y = v2_x*v1_z - v2_z*v1_x;
      this.normal.z = v1_x*v2_y - v1_y*v2_x;
      this.normal.x = x;
      this.normal.y = y;
      this.normal.normalize();
   }

   public Plane3d(Plane3d plane)
   {
      this.point.set(plane.point);
      this.normal.set(plane.normal);
   }

   public void getPoint(Point3d pointToPack)
   {
      pointToPack.set(this.point);
   }
   
   public Point3d getPointCopy()
   {
      Point3d pointToReturn = new Point3d();
      this.getPoint(pointToReturn);
      return pointToReturn;
   }
   
   public void setPoint(Point3d point)
   {
      this.point.set(point);
   }

   public void setPoint(double x, double y, double z)
   {
      point.set(x, y, z); 
   }

   public void setPoints(Point3d pointA, Point3d pointB, Point3d pointC)
   {
      point.set(pointA);
      double v1_x = pointB.x - pointA.x;
      double v1_y = pointB.y - pointA.y;
      double v1_z = pointB.z - pointA.z;
      
      double v2_x = pointC.x - pointA.x;
      double v2_y = pointC.y - pointA.y;
      double v2_z = pointC.z - pointA.z;

      double x = v1_y*v2_z - v1_z*v2_y;
      double y = v2_x*v1_z - v2_z*v1_x;
      this.normal.z = v1_x*v2_y - v1_y*v2_x;
      this.normal.x = x;
      this.normal.y = y;
      this.normal.normalize();
   }
   
   public void getNormal(Vector3d normalToPack)
   {
      normalToPack.set(normal);
   }
   
   public Vector3d getNormalCopy()
   {
      Vector3d normalToReturn = new Vector3d();
      this.getNormal(normalToReturn);
      return normalToReturn;
   }
   
   public void setNormal(double x, double y, double z)
   {
      normal.set(x, y, z);
      normal.normalize();
   }
   
   public void set(Plane3d plane3d)
   {
      this.normal.set(plane3d.normal);
      this.point.set(plane3d.point);
   }

   public void setNormal(Vector3d normal)
   {
      this.normal.set(normal);
   }

   public boolean epsilonEquals(Plane3d plane, double epsilon)
   {
      return ((plane.normal.epsilonEquals(normal, epsilon)) && (plane.point.epsilonEquals(point, epsilon)));
   }

   public boolean isOnOrAbove(Point3d pointToTest)
   {
      return isOnOrAbove(pointToTest, 0.0);
   }

   public boolean isOnOrAbove(Point3d pointToTest, double epsilon)
   {
      temporaryVector.set(pointToTest);
      temporaryVector.sub(this.point);

      return (temporaryVector.dot(this.normal) >= -epsilon);  
   }

   public boolean isOnOrBelow(Point3d pointToTest)
   {
      return isOnOrBelow(pointToTest, 0.0);
   }

   public boolean isOnOrBelow(Point3d pointToTest, double epsilon)
   {
      return isOnOrBelow(pointToTest.x, pointToTest.y, pointToTest.z, epsilon);
   }
   
   public boolean isOnOrBelow(double x, double y, double z, double epsilon)
   {
      temporaryVector.set(x, y, z);
      temporaryVector.sub(this.point);

      return (temporaryVector.dot(this.normal) <= epsilon);
   }

   public Point3d orthogonalProjectionCopy(Point3d point)
   {
      Point3d returnPoint = new Point3d(point);
      orthogonalProjection(returnPoint);
      return returnPoint;
   }
   
   // this method was not tested. Use it at your own risk.
   public void orthogonalProjection(Vector3d vectorToProject)
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
   
   public void orthogonalProjection(Point3d pointToProject)
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

   public double distance(Point3d point)
   {
      temporaryVector.set(point);
      temporaryVector.sub(this.point);
      double temporaryDouble = temporaryVector.dot(this.normal);

      return Math.abs(temporaryDouble);
   }

   public double signedDistance(Point3d point)
   {
      temporaryVector.set(point);
      temporaryVector.sub(this.point);
      double temporaryDouble = temporaryVector.dot(this.normal);

      return temporaryDouble;
   }



   public Plane3d applyTransformCopy(RigidBodyTransform transformation)
   {
      Plane3d returnPlane = new Plane3d(this);
      returnPlane.applyTransform(transformation);

      return returnPlane;
   }
   
   public void applyTransform(RigidBodyTransform transformation)
   {
      transformation.transform(normal);
      transformation.transform(point);
   }

   public void getIntersectionWithLine(Point3d intersectionToPack, Point3d lineStart, Vector3d lineVector)
   {
      // po = line start, p1 = line end
      // v0 = point on plane
      // n = plane normal
      // intersection point is p(s) = p0 + s*(p1 - p0)
      // scalar s = (n dot (v0 - p0))/(n dot (p1 - p0)

      Vector3d fromP0toV0 = new Vector3d(point);
      fromP0toV0.sub(lineStart);

      double scaleFactor = normal.dot(fromP0toV0) / normal.dot(lineVector);
      intersectionToPack.scaleAdd(scaleFactor, lineVector, lineStart);
   }

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

   public void setToNaN()
   {
      setPoint(Double.NaN, Double.NaN, Double.NaN);
      setNormal(Double.NaN, Double.NaN, Double.NaN);
   }
   
   public String toString()
   {
      StringBuilder builder = new StringBuilder();
      
      builder.append("point = " + point + ", normal = " + normal + "\n");
      
      return builder.toString();
   }

}
