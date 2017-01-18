package us.ihmc.robotics.geometry.shapes;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;

public class Ellipsoid3d extends Shape3d<Ellipsoid3d>
{
   private Vector3d radius;

   private final Point3d tempPoint3d;

   public Ellipsoid3d(Ellipsoid3d ellipsoid)
   {
      this(ellipsoid.getXRadius(), ellipsoid.getYRadius(), ellipsoid.getZRadius(), ellipsoid.getTransformUnsafe());
   }

   public Ellipsoid3d(double xRadius, double yRadius, double zRadius)
   {
      radius = new Vector3d(xRadius, yRadius, zRadius);
      
      tempPoint3d = new Point3d();
   }

   public Ellipsoid3d(double xRadius, double yRadius, double zRadius, RigidBodyTransform transform)
   {
      setTransform(transform);
      radius = new Vector3d(xRadius, yRadius, zRadius);
      
      tempPoint3d = new Point3d();
   }

   public void getCenter(Point3d centerToPack)
   {
      getPosition(centerToPack);
   }
   
   public void getRadii(Vector3d radiiToPack)
   {
      radiiToPack.set(radius);
   }

   public double getXRadius()
   {
      return radius.getX();
   }

   public void setXRadius(double xRadius)
   {
      radius.setX(xRadius);
   }

   public double getYRadius()
   {
      return radius.getY();
   }

   public void setYRadius(double yRadius)
   {
      radius.setY(yRadius);
   }

   public double getZRadius()
   {
      return radius.getZ();
   }

   public void setZRadius(double zRadius)
   {
      radius.setZ(zRadius);
   }

   @Override
   protected boolean checkIfInsideShapeFrame(Point3d pointToCheck, Point3d closestPointToPack, Vector3d normalToPack)
   {
      tempPoint3d.set(pointToCheck);
      double scaledX = tempPoint3d.getX() / radius.getX();
      double scaledY = tempPoint3d.getY() / radius.getY();
      double scaledZ = tempPoint3d.getZ() / radius.getZ();
      
      double sumOfSquares = scaledX * scaledX + scaledY * scaledY + scaledZ * scaledZ;
      boolean isInside1 = sumOfSquares <= 1.0;
      
      if (sumOfSquares > 1e-10)
      {
         double scaleFactor = 1.0 / Math.sqrt(sumOfSquares);
         scaledX = scaledX * scaleFactor;
         scaledY = scaledY * scaleFactor;
         scaledZ = scaledZ * scaleFactor;
      }
      else
      {
         scaledX = 1.0;
         scaledY = 0.0;
         scaledZ = 0.0;
      }
      
      closestPointToPack.set(scaledX * radius.getX(), scaledY * radius.getY(), scaledZ * radius.getZ());
      
      double normalX = 2.0 * closestPointToPack.getX() / (radius.getX() * radius.getX());
      double normalY = 2.0 * closestPointToPack.getY() / (radius.getY() * radius.getY());
      double normalZ = 2.0 * closestPointToPack.getZ() / (radius.getZ() * radius.getZ());
      
      normalToPack.set(normalX, normalY, normalZ);
      normalToPack.normalize();
      return isInside1;
   }

   @Override
   protected boolean isInsideOrOnSurfaceShapeFrame(Point3d point, double epsilon)
   {
      tempPoint3d.set(point);
      double scaledX = tempPoint3d.getX() / radius.getX();
      double scaledY = tempPoint3d.getY() / radius.getY();
      double scaledZ = tempPoint3d.getZ() / radius.getZ();
      
      double sumOfSquares = scaledX * scaledX + scaledY * scaledY + scaledZ * scaledZ;
      
      return sumOfSquares <= 1.0 + epsilon;
   }

   @Override
   protected double distanceShapeFrame(Point3d point)
   {
      tempPoint3d.set(point);
      orthogonalProjectionShapeFrame(tempPoint3d);
      return tempPoint3d.distance(point);
   }

   @Override
   protected void orthogonalProjectionShapeFrame(Point3d pointToCheckAndPack)
   {
      double scaledX = pointToCheckAndPack.getX() / radius.getX();
      double scaledY = pointToCheckAndPack.getY() / radius.getY();
      double scaledZ = pointToCheckAndPack.getZ() / radius.getZ();
      
      double sumOfSquares = scaledX * scaledX + scaledY * scaledY + scaledZ * scaledZ;
      boolean isInside = sumOfSquares <= 1.0;
      
      if (!isInside)
      {
         double scaleFactor = 1.0 / Math.sqrt(sumOfSquares);
         scaledX = scaledX * scaleFactor;
         scaledY = scaledY * scaleFactor;
         scaledZ = scaledZ * scaleFactor;
         pointToCheckAndPack.set(scaledX * radius.getX(), scaledY * radius.getY(), scaledZ * radius.getZ());
      }
   }
   
   @Override
   public void set(Ellipsoid3d other)
   {
      if (this != other)
      {
         setTransform(other.getTransformUnsafe());
         radius.set(other.radius);
      }
   }

   @Override
   public void setToZero()
   {
      radius.set(0.0, 0.0, 0.0);
   }

   @Override
   public void setToNaN()
   {
      radius.set(Double.NaN, Double.NaN, Double.NaN);
   }

   @Override
   public boolean containsNaN()
   {
      return Double.isNaN(radius.getX()) || Double.isNaN(radius.getY()) || Double.isNaN(radius.getZ());
   }

   @Override
   public boolean epsilonEquals(Ellipsoid3d other, double epsilon)
   {
      return radius.epsilonEquals(other.radius, epsilon);
   }

   @Override
   public String toString()
   {
      return "radius = " + radius + ", \ntransform = " + getTransformUnsafe() + "\n";
   }
}
