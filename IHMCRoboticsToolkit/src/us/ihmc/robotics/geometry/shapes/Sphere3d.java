package us.ihmc.robotics.geometry.shapes;

import us.ihmc.robotics.geometry.RigidBodyTransform;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class Sphere3d implements Shape3d
{
   private final Point3d center = new Point3d();
   private double radius = 1.0;

   private final Point3d temporaryPoint = new Point3d();
   public final Vector3d temporaryVector = new Vector3d();

   public Sphere3d()
   {
   }

   public Sphere3d(double radius)
   {
      this.setRadius(radius);
   }

   public Sphere3d(Point3d center, double radius)
   {
      this.center.set(center);
      this.setRadius(radius);
   }

   public Sphere3d(Sphere3d sphere3d)
   {
      this.center.set(sphere3d.center);
      this.radius = sphere3d.radius;
   }

   public double getRadius()
   {
      return radius;
   }

   public void setRadius(double radius)
   {
      this.radius = radius;
   }

   public void getCenter(Point3d centerToPack)
   {
      centerToPack.set(center);
   }

   public void set(Sphere3d sphere3d)
   {
      this.center.set(sphere3d.center);
      this.radius = sphere3d.radius;
   }

   public void applyTransform(RigidBodyTransform transform3D)
   {
      transform3D.transform(this.center);
   }

   public boolean isInsideOrOnSurface(Point3d pointToCheck)
   {
      return isInsideOrOnSurface(pointToCheck, 0.0);
   }

   public boolean isInsideOrOnSurface(Point3d pointToCheck, double epsilon)
   {
      return (pointToCheck.distanceSquared(center) <= (radius + epsilon) * (radius + epsilon));
   }

   public double distance(Point3d point)
   {
      temporaryPoint.set(point);
      orthogonalProjection(temporaryPoint);

      return temporaryPoint.distance(point);
   }

   public void orthogonalProjection(Point3d point)
   {
      temporaryVector.set(point);
      temporaryVector.sub(this.center);

      double distance = temporaryVector.length();

      if (distance >= radius)
      {
         temporaryVector.scale(radius / distance);
         point.set(center);
         point.add(temporaryVector);
      }

   }

   public boolean checkIfInside(Point3d pointInWorldToCheck, Point3d closestPointToPack, Vector3d normalToPack)
   {
      boolean isInside = pointInWorldToCheck.distanceSquared(center) <= radius * radius;

      surfaceNormalAt(pointInWorldToCheck, normalToPack);

      temporaryVector.set(normalToPack);
      temporaryVector.scale(radius);

      closestPointToPack.set(center);
      closestPointToPack.add(temporaryVector);

      return isInside;
   }

   private void surfaceNormalAt(Point3d pointToCheck, Vector3d normalToPack)
   {
      normalToPack.set(pointToCheck);
      normalToPack.sub(this.center);

      double distance = normalToPack.length();

      if (distance > 1e-7)
      {
         normalToPack.scale(1.0 / distance);
      }
      else
      {
         normalToPack.set(0.0, 0.0, 1.0);
      }
   }


   public String toString()
   {
      return "center = " + center + ", radius = " + radius + "\n";
   }


}
