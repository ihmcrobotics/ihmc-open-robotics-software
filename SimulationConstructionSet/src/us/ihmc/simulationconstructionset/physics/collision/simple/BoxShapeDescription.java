package us.ihmc.simulationconstructionset.physics.collision.simple;

import javax.vecmath.Point3d;

import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;

public class BoxShapeDescription<T extends BoxShapeDescription<T>> implements CollisionShapeDescription<T>
{
   private double halfLengthX;
   private double halfWidthY;
   private double halfHeightZ;

   private final RigidBodyTransform transform = new RigidBodyTransform();

   private final BoundingBox3d boundingBox = new BoundingBox3d(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY,
                                                               Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
   private boolean boundingBoxNeedsUpdating = true;

   public BoxShapeDescription(double halfLengthX, double halfWidthY, double halfHeightZ)
   {
      this.halfLengthX = halfLengthX;
      this.halfWidthY = halfWidthY;
      this.halfHeightZ = halfHeightZ;
      boundingBoxNeedsUpdating = true;
   }

   @Override
   public BoxShapeDescription<T> copy()
   {
      BoxShapeDescription<T> copy = new BoxShapeDescription<T>(halfLengthX, halfWidthY, halfHeightZ);
      copy.transform.set(this.transform);
      copy.boundingBox.set(this.boundingBox);
      return copy;
   }

   public double getHalfLengthX()
   {
      return halfLengthX;
   }

   public double getHalfWidthY()
   {
      return halfWidthY;
   }

   public double getHalfHeightZ()
   {
      return halfHeightZ;
   }

   public void getTransform(RigidBodyTransform transformToPack)
   {
      transformToPack.set(transform);
   }

   @Override
   public void applyTransform(RigidBodyTransform transformToWorld)
   {
      transform.multiply(transformToWorld, transform);
      boundingBoxNeedsUpdating = true;
   }

   @Override
   public void setFrom(T box)
   {
      this.halfLengthX = box.getHalfLengthX();
      this.halfWidthY = box.getHalfWidthY();
      this.halfHeightZ = box.getHalfHeightZ();

      box.getTransform(this.transform);
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
      throw new RuntimeException("Implement Me!");      
   }

   @Override
   public boolean isPointInside(Point3d pointInWorld)
   {
      throw new RuntimeException("Implement Me!");
   }
}
