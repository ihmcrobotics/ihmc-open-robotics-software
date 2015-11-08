package us.ihmc.robotics.geometry.shapes;

import us.ihmc.robotics.geometry.RigidBodyTransform;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class Ellipsoid3d implements Shape3d
{
   private RigidBodyTransform transform = new RigidBodyTransform();
   private RigidBodyTransform inverseTransform = new RigidBodyTransform();
   private double xRadius, yRadius, zRadius;

   private final Point3d tempPoint3d = new Point3d();

   public Ellipsoid3d(double xRadius, double yRadius, double zRadius)
   {
      this.xRadius = xRadius;
      this.yRadius = yRadius;
      this.zRadius = zRadius;
   }

   public Ellipsoid3d(double xRadius, double yRadius, double zRadius, RigidBodyTransform transform)
   {
      this(xRadius, yRadius, zRadius);
      this.transform.set(transform);
      this.inverseTransform.set(transform);
      this.inverseTransform.invert();
   }

   public Ellipsoid3d(Ellipsoid3d ellipsoid)
   {
      this(ellipsoid.getXRadius(), ellipsoid.getYRadius(), ellipsoid.getZRadius(), ellipsoid.transform);
   }

   public void getCenter(Point3d centerToPack)
   {
      centerToPack.set(0.0, 0.0, 0.0);
      transform.transform(centerToPack);
   }
   
   public void getRadii(Vector3d radiiToPack)
   {
      radiiToPack.set(xRadius, yRadius, zRadius);
   }

   public double getXRadius()
   {
      return xRadius;
   }

   public void setXRadius(double xRadius)
   {
      this.xRadius = xRadius;
   }

   public double getYRadius()
   {
      return yRadius;
   }

   public void setYRadius(double yRadius)
   {
      this.yRadius = yRadius;
   }

   public double getZRadius()
   {
      return zRadius;
   }

   public void setZRadius(double zRadius)
   {
      this.zRadius = zRadius;
   }

   public void setTransform(RigidBodyTransform newTransform)
   {
      transform.set(newTransform);
      inverseTransform.set(transform);
      inverseTransform.invert();
   }

   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   public void applyTransform(RigidBodyTransform transform)
   {
      tempTransform.set(transform);
      tempTransform.multiply(this.transform);
      this.transform.set(tempTransform);
      inverseTransform.set(this.transform);
      inverseTransform.invert();
   }

   private final Point3d tempPointForDistance = new Point3d();
   public double distance(Point3d point)
   {
      tempPointForDistance.set(point);
      orthogonalProjection(tempPointForDistance);
      return tempPointForDistance.distance(point);
   }

   public boolean checkIfInside(Point3d pointToCheck, Point3d closestPointToPack, Vector3d normalToPack)
   {
      tempPoint3d.set(pointToCheck);
      inverseTransform.transform(tempPoint3d);
      boolean isInside = checkIfInsideLocal(tempPoint3d, closestPointToPack, normalToPack);
      transform.transform(closestPointToPack);
      transform.transform(normalToPack);

      return isInside;
   }

   private boolean checkIfInsideLocal(Point3d pointToCheck, Point3d closestPointToPack, Vector3d normalToPack)
   {
      double scaledX = pointToCheck.getX() / xRadius;
      double scaledY = pointToCheck.getY() / yRadius;
      double scaledZ = pointToCheck.getZ() / zRadius;

      double sumOfSquares = scaledX * scaledX + scaledY * scaledY + scaledZ * scaledZ;
      boolean isInside = sumOfSquares <= 1.0;

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

      closestPointToPack.set(scaledX * xRadius, scaledY * yRadius, scaledZ * zRadius);
      
      double normalX = 2.0 * closestPointToPack.getX() / (xRadius * xRadius);
      double normalY = 2.0 * closestPointToPack.getY() / (yRadius * yRadius);
      double normalZ = 2.0 * closestPointToPack.getZ() / (zRadius * zRadius);

      normalToPack.set(normalX, normalY, normalZ);
      normalToPack.normalize();

      return isInside;
   }

   public boolean isInsideOrOnSurface(Point3d pointToCheck)
   {
      return isInsideOrOnSurface(pointToCheck, 0.0);
   }

   public boolean isInsideOrOnSurface(Point3d point, double epsilon)
   {
      tempPoint3d.set(point);
      inverseTransform.transform(tempPoint3d);
      boolean isInside = isInsideOrOnSurfaceLocal(tempPoint3d, epsilon);

      return isInside;
   }

   public boolean isInsideOrOnSurfaceLocal(Point3d pointToCheck, double epsilon)
   {
      double scaledX = pointToCheck.getX() / xRadius;
      double scaledY = pointToCheck.getY() / yRadius;
      double scaledZ = pointToCheck.getZ() / zRadius;

      double sumOfSquares = scaledX * scaledX + scaledY * scaledY + scaledZ * scaledZ;

      return sumOfSquares <= 1.0 + epsilon;
   }

   public void orthogonalProjection(Point3d pointToCheckAndPack)
   {
      inverseTransform.transform(pointToCheckAndPack);
      orthogonalProjectionLocal(pointToCheckAndPack);
      transform.transform(pointToCheckAndPack);
   }
   
   private void orthogonalProjectionLocal(Point3d pointToCheckAndPack)
   {
      double scaledX = pointToCheckAndPack.getX() / xRadius;
      double scaledY = pointToCheckAndPack.getY() / yRadius;
      double scaledZ = pointToCheckAndPack.getZ() / zRadius;

      double sumOfSquares = scaledX * scaledX + scaledY * scaledY + scaledZ * scaledZ;
      boolean isInside = sumOfSquares <= 1.0;

      if (!isInside)
      {
         double scaleFactor = 1.0 / Math.sqrt(sumOfSquares);
         scaledX = scaledX * scaleFactor;
         scaledY = scaledY * scaleFactor;
         scaledZ = scaledZ * scaleFactor;
         pointToCheckAndPack.set(scaledX * xRadius, scaledY * yRadius, scaledZ * zRadius);
      }
   }
   
   @Override
   public String toString()
   {
      return "xRadius = " + xRadius + ", yRadius = " + yRadius + ", zRadius = " + zRadius + ", \ntransform = " + transform + "\n";
   }
}
