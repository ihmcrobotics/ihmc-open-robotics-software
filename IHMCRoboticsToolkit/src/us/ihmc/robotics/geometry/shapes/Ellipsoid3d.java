package us.ihmc.robotics.geometry.shapes;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.robotics.geometry.transformables.AbstractPose;

public class Ellipsoid3d extends Shape3d<Ellipsoid3d>
{
   private Vector3D radius;

   private final Point3D tempPoint3d;

   public Ellipsoid3d(Ellipsoid3d ellipsoid)
   {
      this(ellipsoid.getXRadius(), ellipsoid.getYRadius(), ellipsoid.getZRadius(), ellipsoid);
   }

   public Ellipsoid3d(double xRadius, double yRadius, double zRadius)
   {
      radius = new Vector3D(xRadius, yRadius, zRadius);
      
      tempPoint3d = new Point3D();
   }

   public Ellipsoid3d(double xRadius, double yRadius, double zRadius, RigidBodyTransform transform)
   {
      setPose(transform);
      radius = new Vector3D(xRadius, yRadius, zRadius);
      
      tempPoint3d = new Point3D();
   }

   public Ellipsoid3d(double xRadius, double yRadius, double zRadius, AbstractPose pose)
   {
      setPose(pose);
      radius = new Vector3D(xRadius, yRadius, zRadius);
      
      tempPoint3d = new Point3D();
   }

   public void getCenter(Point3DBasics centerToPack)
   {
      getPosition(centerToPack);
   }
   
   public void getRadii(Vector3DBasics radiiToPack)
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
   protected boolean checkIfInsideShapeFrame(Point3DReadOnly pointToCheck, Point3DBasics closestPointToPack, Vector3DBasics normalToPack)
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
   protected boolean isInsideOrOnSurfaceShapeFrame(Point3DReadOnly point, double epsilon)
   {
      tempPoint3d.set(point);
      double scaledX = tempPoint3d.getX() / radius.getX();
      double scaledY = tempPoint3d.getY() / radius.getY();
      double scaledZ = tempPoint3d.getZ() / radius.getZ();
      
      double sumOfSquares = scaledX * scaledX + scaledY * scaledY + scaledZ * scaledZ;
      
      return sumOfSquares <= 1.0 + epsilon;
   }

   @Override
   protected double distanceShapeFrame(Point3DReadOnly point)
   {
      tempPoint3d.set(point);
      orthogonalProjectionShapeFrame(tempPoint3d);
      return tempPoint3d.distance(point);
   }

   @Override
   protected void orthogonalProjectionShapeFrame(Point3DBasics pointToCheckAndPack)
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
         setPose(other);
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
      return "radius = " + radius + ", \npose = " + getPoseString() + "\n";
   }

   @Override
   public void applyTransform(Transform transform)
   {
      applyTransformToPose(transform);
   }
}
