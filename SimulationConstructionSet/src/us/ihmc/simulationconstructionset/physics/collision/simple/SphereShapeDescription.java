package us.ihmc.simulationconstructionset.physics.collision.simple;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;

public class SphereShapeDescription<T extends SphereShapeDescription<T>> implements CollisionShapeDescription<T>
{
   private double radius;
   private Point3D center = new Point3D();

   private final BoundingBox3D boundingBox = new BoundingBox3D(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY,
                                                               Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
   private boolean boundingBoxNeedsUpdating = true;

   public SphereShapeDescription(double radius, Point3D center)
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

   public void getCenter(Point3D centerToPack)
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
   public void getBoundingBox(BoundingBox3D boundingBoxToPack)
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
   public boolean isPointInside(Point3D pointInWorld)
   {
      return (center.distanceSquared(pointInWorld) <= radius * radius);
   }

   private final Vector3D tempVectorForRolling = new Vector3D();

   @Override
   public boolean rollContactIfRolling(Vector3D surfaceNormal, Point3D pointToRoll)
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
