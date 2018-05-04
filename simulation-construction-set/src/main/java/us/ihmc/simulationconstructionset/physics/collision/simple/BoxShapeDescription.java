package us.ihmc.simulationconstructionset.physics.collision.simple;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;

public class BoxShapeDescription<T extends BoxShapeDescription<T>> implements CollisionShapeDescription<T>
{
   private double halfLengthX;
   private double halfWidthY;
   private double halfHeightZ;

   private final RigidBodyTransform transform = new RigidBodyTransform();

   private final BoundingBox3D boundingBox = new BoundingBox3D(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY,
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
      transform.preMultiply(transformToWorld);
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
      throw new RuntimeException("Implement Me!");      
   }

   @Override
   public boolean isPointInside(Point3D pointInWorld)
   {
      throw new RuntimeException("Implement Me!");
   }

   /**
    * Box shape will not roll, so this method always returns false.
    * @return false
    */
   @Override
   public boolean rollContactIfRolling(Vector3D surfaceNormal, Point3D pointToRoll)
   { 
      return false;
   }
}
