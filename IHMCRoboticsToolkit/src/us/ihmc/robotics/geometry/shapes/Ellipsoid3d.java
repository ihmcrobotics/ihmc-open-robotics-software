package us.ihmc.robotics.geometry.shapes;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public class Ellipsoid3d extends Shape3d<Ellipsoid3d>
{
   private final Vector3D radius = new Vector3D();

   public Ellipsoid3d(Ellipsoid3d other)
   {
      setPose(other);
      radius.set(other.radius);
   }

   public Ellipsoid3d(double xRadius, double yRadius, double zRadius)
   {
      radius.set(xRadius, yRadius, zRadius);
   }

   public Ellipsoid3d(double xRadius, double yRadius, double zRadius, RigidBodyTransform transform)
   {
      setPose(transform);
      radius.set(xRadius, yRadius, zRadius);
   }

   public Ellipsoid3d(double xRadius, double yRadius, double zRadius, Pose3D pose)
   {
      setPose(pose);
      radius.set(xRadius, yRadius, zRadius);
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
   protected boolean checkIfInsideShapeFrame(double x, double y, double z, Point3DBasics closestPointToPack, Vector3DBasics normalToPack)
   {
      double sumOfSquares = computeSumOfScaledSquared(x, y, z);

      boolean isInside = sumOfSquares <= 1.0;

      if (sumOfSquares > 1e-10)
      {
         double scaleFactor = 1.0 / Math.sqrt(sumOfSquares);
         closestPointToPack.set(x, y, z);
         closestPointToPack.scale(scaleFactor);
      }
      else
      {
         closestPointToPack.setToZero();
         closestPointToPack.setX(radius.getX());
      }

      double rxSquared = radius.getX() * radius.getX();
      double rySquared = radius.getY() * radius.getY();
      double rzSquared = radius.getZ() * radius.getZ();

      normalToPack.set(closestPointToPack);
      normalToPack.scale(1.0 / rxSquared, 1.0 / rySquared, 1.0 / rzSquared);
      normalToPack.normalize();
      return isInside;
   }

   public double computeSumOfScaledSquared(double x, double y, double z)
   {
      return EuclidCoreTools.normSquared(x / radius.getX(), y / radius.getY(), z / radius.getZ());
   }

   @Override
   protected boolean isInsideOrOnSurfaceShapeFrame(double x, double y, double z, double epsilon)
   {
      double scaledX = x / (radius.getX() + epsilon);
      double scaledY = y / (radius.getY() + epsilon);
      double scaledZ = z / (radius.getZ() + epsilon);

      return EuclidCoreTools.normSquared(scaledX, scaledY, scaledZ) <= 1.0;
   }

   @Override
   protected double distanceShapeFrame(double x, double y, double z)
   {
      double sumOfSquares = computeSumOfScaledSquared(x, y, z);

      if (sumOfSquares > 1e-10)
      {
         double scaleFactor = 1.0 - 1.0 / Math.sqrt(sumOfSquares);
         return Math.sqrt(EuclidCoreTools.normSquared(x, y, z)) * scaleFactor;
      }
      else
      {
         return x - radius.getX();
      }
   }

   @Override
   protected void orthogonalProjectionShapeFrame(double x, double y, double z, Point3DBasics projectionToPack)
   {
      double sumOfSquares = computeSumOfScaledSquared(x, y, z);

      boolean isInside = sumOfSquares <= 1.0;

      if (!isInside)
      {
         double scaleFactor = 1.0 / Math.sqrt(sumOfSquares);
         projectionToPack.set(x, y, z);
         projectionToPack.scale(scaleFactor);
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
      super.setToZero();
      radius.setToZero();
   }

   @Override
   public void setToNaN()
   {
      super.setToNaN();
      radius.setToNaN();
   }

   @Override
   public boolean containsNaN()
   {
      return super.containsNaN() || radius.containsNaN();
   }

   @Override
   public boolean epsilonEquals(Ellipsoid3d other, double epsilon)
   {
      return radius.epsilonEquals(other.radius, epsilon) && super.epsilonEqualsPose(other, epsilon);
   }

   @Override
   public String toString()
   {
      return "radius = " + radius + ", \npose = " + getPoseString() + "\n";
   }
}
