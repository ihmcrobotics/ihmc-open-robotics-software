package us.ihmc.robotics.geometry.shapes;

import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.TransformTools;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class Cylinder3d implements Shape3d
{
   private RigidBodyTransform transform = new RigidBodyTransform();
   private double radius;
   private double height;

   private final RigidBodyTransform inverseTransform = new RigidBodyTransform();
   private final Point3d temporaryPoint = new Point3d();


   public Cylinder3d(double height, double radius)
   {
      this(new RigidBodyTransform(), height, radius);
   }

   public Cylinder3d(RigidBodyTransform transform, double height, double radius)
   {
      this.transform.set(transform);
      inverseTransform.set(transform);
      inverseTransform.invert();

      this.height = height;
      this.radius = radius;
   }

   public Cylinder3d(Cylinder3d cylinder3d)
   {
      this(cylinder3d.transform, cylinder3d.radius, cylinder3d.height);
   }

   public void set(Cylinder3d cylinder3d)
   {
      this.transform.set(transform);
      this.height = cylinder3d.height;
      this.radius = cylinder3d.radius;
   }


   public double getRadius()
   {
      return radius;
   }

   public void setRadius(double radius)
   {
      this.radius = radius;
   }

   public double getHeight()
   {
      return height;
   }

   public void setHeight(double height)
   {
      this.height = height;
   }

   public void getTransform(RigidBodyTransform transform)
   {
      transform.set(this.transform);
   }

   public void setTransform(RigidBodyTransform newTransform)
   {
      transform = new RigidBodyTransform(newTransform);
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

   private final Point3d tempPointForCheckInside = new Point3d();

   public boolean checkIfInside(Point3d pointToCheck, Point3d closestPointOnSurfaceToPack, Vector3d normalToPack)
   {
      tempPointForCheckInside.set(pointToCheck);
      inverseTransform.transform(tempPointForCheckInside);

      boolean isInside = checkIfInsideLocal(tempPointForCheckInside, closestPointOnSurfaceToPack, normalToPack);

      transform.transform(closestPointOnSurfaceToPack);
      transform.transform(normalToPack);

      return (isInside);
   }

   public double distance(Point3d point)
   {
      temporaryPoint.set(point);
      orthogonalProjection(temporaryPoint);

      return temporaryPoint.distance(point);
   }

   public boolean isInsideOrOnSurface(Point3d pointToCheck)
   {
      return isInsideOrOnSurface(pointToCheck, 0.0);
   }

   public boolean isInsideOrOnSurface(Point3d pointToCheck, double epsilonToGrowObject)
   {
      temporaryPoint.set(pointToCheck);
      inverseTransform.transform(temporaryPoint);

      return isInsideOrOnSurfaceLocal(temporaryPoint, epsilonToGrowObject);
   }

   private boolean isInsideOrOnSurfaceLocal(Point3d pointToCheck, double epsilonToGrowObject)
   {
      // 0. given point to test P (x,y,z)
      // 1. Transform input point into a cylinder fixed frame. Use transform3d.transform(pointToCheck)
      // 2. Check if z >= 0 and z <= height;
      // 3. Check if (x, y) magnitude is less than radius.

      if (pointToCheck.getZ() < 0.0 - epsilonToGrowObject)
         return false;
      if (pointToCheck.getZ() > height + epsilonToGrowObject)
         return false;

      double radiusSquared = (radius + epsilonToGrowObject) * (radius + epsilonToGrowObject);
      double xySquared = pointToCheck.getX() * pointToCheck.getX() + pointToCheck.getY() * pointToCheck.getY();

      if (xySquared > radiusSquared)
         return false;

      return true;
   }

   private boolean checkIfInsideLocal(Point3d pointToCheck, Point3d closestPointOnSurfaceToPack, Vector3d normalToPack)
   {
      boolean insideOrOnSurfaceLocal = true;

      closestPointOnSurfaceToPack.set(pointToCheck);

      if (closestPointOnSurfaceToPack.getZ() < 0.0)
      {
         closestPointOnSurfaceToPack.setZ(0.0);
         insideOrOnSurfaceLocal = false;
      }

      if (closestPointOnSurfaceToPack.getZ() > height)
      {
         closestPointOnSurfaceToPack.setZ(height);
         insideOrOnSurfaceLocal = false;
      }

      double xyLengthSquared = closestPointOnSurfaceToPack.getX() * closestPointOnSurfaceToPack.getX()
                               + closestPointOnSurfaceToPack.getY() * closestPointOnSurfaceToPack.getY();
      if (xyLengthSquared > (radius + 1e-10) * (radius + 1e-10))
      {
         double xyLength = Math.sqrt(xyLengthSquared);
         double scale = radius / xyLength;

         closestPointOnSurfaceToPack.setX(closestPointOnSurfaceToPack.getX() * scale);
         closestPointOnSurfaceToPack.setY(closestPointOnSurfaceToPack.getY() * scale);
         insideOrOnSurfaceLocal = false;
      }

      if (insideOrOnSurfaceLocal)
      {
         double distanceSquaredToSide = radius * radius - xyLengthSquared;
         double distanceSquaredToTop = (height - pointToCheck.getZ()) * (height - pointToCheck.getZ());
         double distanceSquaredToBottom = pointToCheck.getZ() * pointToCheck.getZ();

         if ((distanceSquaredToSide < distanceSquaredToTop) && (distanceSquaredToSide < distanceSquaredToBottom))
         {
            double xyLength = Math.sqrt(xyLengthSquared);
            if (xyLength > 1e-10)
            {
               double scale = radius / xyLength;

               closestPointOnSurfaceToPack.setX(closestPointOnSurfaceToPack.getX() * scale);
               closestPointOnSurfaceToPack.setY(closestPointOnSurfaceToPack.getY() * scale);
               normalToPack.set(pointToCheck.getX(), pointToCheck.getY(), 0.0);
               normalToPack.normalize();
            }
            else
            {
               closestPointOnSurfaceToPack.setX(radius);
               closestPointOnSurfaceToPack.setY(0.0);
               normalToPack.set(1.0, 0.0, 0.0);
            }
         }
         else if (distanceSquaredToTop < distanceSquaredToBottom)
         {
            closestPointOnSurfaceToPack.setZ(height);
            normalToPack.set(0.0, 0.0, 1.0);
         }
         else
         {
            closestPointOnSurfaceToPack.setZ(0.0);
            normalToPack.set(0.0, 0.0, -1.0);
         }
      }
      else
      {
         normalToPack.set(pointToCheck);
         normalToPack.sub(closestPointOnSurfaceToPack);
         normalToPack.normalize();
      }

      return insideOrOnSurfaceLocal;
   }
   
   public void orthogonalProjection(Point3d pointToCheckAndPack)
   {
      inverseTransform.transform(pointToCheckAndPack);
      orthogonalProjectionLocal(pointToCheckAndPack);
      transform.transform(pointToCheckAndPack);
   }

   private void orthogonalProjectionLocal(Point3d pointToCheckAndPack)
   {
      if (pointToCheckAndPack.getZ() < 0.0)
         pointToCheckAndPack.setZ(0.0);
      if (pointToCheckAndPack.getZ() > height)
         pointToCheckAndPack.setZ(height);

      double xyLengthSquared = pointToCheckAndPack.getX() * pointToCheckAndPack.getX() + pointToCheckAndPack.getY() * pointToCheckAndPack.getY();
      if (xyLengthSquared > radius * radius)
      {
         double xyLength = Math.sqrt(xyLengthSquared);
         double scale = radius / xyLength;

         pointToCheckAndPack.setX(pointToCheckAndPack.getX() * scale);
         pointToCheckAndPack.setY(pointToCheckAndPack.getY() * scale);
      }
   }

   public Plane3d getPlane(CylinderFaces face)
   {
      Plane3d plane3d;

      Matrix3d m1 = new Matrix3d();
      transform.get(m1);
      Vector3d normal = new Vector3d();
      m1.getColumn(Direction.Z.getIndex(), normal);

      switch (face)
      {
         case TOP :
            RigidBodyTransform topTransform = TransformTools.transformLocalZ(transform, height);
            Vector3d topCenter = new Vector3d();
            topTransform.get(topCenter);
            Point3d topCenterPoint = new Point3d(topCenter);

            plane3d = new Plane3d(topCenterPoint, normal);

            break;

         case BOTTOM :
            Vector3d bottomCenter = new Vector3d();
            transform.get(bottomCenter);
            Point3d bottomCenterPoint = new Point3d(bottomCenter);
            normal.negate();

            plane3d = new Plane3d(bottomCenterPoint, normal);

            break;

         default :
            throw(new RuntimeException("Unrecognized cylinder face"));
      }


      return plane3d;
   }

   public static enum CylinderFaces {TOP, BOTTOM}

   @Override
   public String toString()
   {
      return "height = " + height + ", radius = " + radius + ", transform = " + transform + "\n";
   }

}
