package us.ihmc.simulationconstructionset.physics.collision.simple;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;

public class SphereShapeDescription<T extends SphereShapeDescription<T>> implements CollisionShapeDescription<T>
{
   private double radius;
   private Point3d center = new Point3d();

   private final BoundingBox3d boundingBox = new BoundingBox3d(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY,
                                                               Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
   private boolean boundingBoxNeedsUpdating = true;

   public SphereShapeDescription(double radius, Point3d center)
   {
      this.radius = radius;
      this.center.set(center);
      boundingBoxNeedsUpdating = true;
   }

   @Override
   public SphereShapeDescription<T> copy()
   {
      SphereShapeDescription<T> copy = new SphereShapeDescription<T>(radius, center);
      boundingBoxNeedsUpdating = true;
      return copy;
   }

   public double getRadius()
   {
      return radius;
   }

   public void getCenter(Point3d centerToPack)
   {
      centerToPack.set(center);
   }

   @Override
   public void setFrom(T sphereShapeDescription)
   {
      this.radius = sphereShapeDescription.getRadius();
      sphereShapeDescription.getCenter(this.center);
      boundingBoxNeedsUpdating = true;
   }

   @Override
   public void applyTransform(RigidBodyTransform transform)
   {
      transform.transform(center);
      boundingBoxNeedsUpdating = true;
   }

   @Override
   public void getBoundingBox(BoundingBox3d boundingBoxToPack)
   {
      if (boundingBoxNeedsUpdating)
      {
         updateBoundingBox();
         boundingBoxNeedsUpdating = false;
      }

      boundingBoxToPack.set(boundingBox);
   }

   private void updateBoundingBox()
   {
      boundingBox.set(center.getX() - radius, center.getY() - radius, center.getZ() - radius, center.getX() + radius, center.getY() + radius,
                      center.getZ() + radius);
   }

   @Override
   public boolean isPointInside(Point3d pointInWorld)
   {
      return (center.distanceSquared(pointInWorld) <= radius * radius);
   }

   private final Vector3d tempVectorForRolling = new Vector3d();

   @Override
   public boolean rollContactIfRolling(Vector3d surfaceNormal, Point3d pointToRoll)
   {
      pointToRoll.set(center);
      tempVectorForRolling.set(surfaceNormal);
      tempVectorForRolling.normalize();
      tempVectorForRolling.scale(radius);
      pointToRoll.add(tempVectorForRolling);

      //TODO: Not necessarily true all the time...
      return true;
   }

}
