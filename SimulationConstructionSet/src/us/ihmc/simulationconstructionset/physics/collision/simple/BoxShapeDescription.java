package us.ihmc.simulationconstructionset.physics.collision.simple;

import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;

public class BoxShapeDescription<T extends BoxShapeDescription<T>> implements CollisionShapeDescription<T>
{
   private double halfLengthX;
   private double halfWidthY;
   private double halfHeightZ;

   private final RigidBodyTransform transform = new RigidBodyTransform();

   private final BoundingBox3d boundingBox = new BoundingBox3d(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

   public BoxShapeDescription(double halfLengthX, double halfWidthY, double halfHeightZ)
   {
      this.halfLengthX = halfLengthX;
      this.halfWidthY = halfWidthY;
      this.halfHeightZ = halfHeightZ;
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
//      boundingBox.set(-halfLengthX, -halfWidthY, -halfHeightZ, halfLengthX, halfWidthY, halfHeightZ);
//      boundingBox.transform
   }

   @Override
   public void applyTransform(RigidBodyTransform transformToWorld)
   {
      transform.multiply(transformToWorld, transform);
   }

   @Override
   public void setFrom(T box)
   {
      this.halfLengthX = box.getHalfLengthX();
      this.halfWidthY = box.getHalfWidthY();
      this.halfHeightZ = box.getHalfHeightZ();

      box.getTransform(this.transform);
      this.boundingBox.set(box.getBoundingBox());
   }

   @Override
   public BoundingBox3d getBoundingBox()
   {
      return boundingBox;
   }
}
