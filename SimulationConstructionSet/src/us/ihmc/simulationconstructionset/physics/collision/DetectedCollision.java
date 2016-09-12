package us.ihmc.simulationconstructionset.physics.collision;

import javax.vecmath.Point3d;

import us.ihmc.simulationconstructionset.physics.CollisionShape;

public class DetectedCollision
{
   private final CollisionShape shapeA;
   private final CollisionShape shapeB;
   private final Point3d pointOnA = new Point3d();
   private final Point3d pointOnB = new Point3d();

   public DetectedCollision(CollisionShape shapeA, CollisionShape shapeB, Point3d pointOnA, Point3d pointOnB)
   {
      this.shapeA = shapeA;
      this.shapeB = shapeB;

      this.pointOnA.set(pointOnA);
      this.pointOnB.set(pointOnB);
   }

   public CollisionShape getShapeA()
   {
      return shapeA;
   }

   public CollisionShape getShapeB()
   {
      return shapeB;
   }

   public void getPointOnA(Point3d pointOnAToPack)
   {
      pointOnAToPack.set(pointOnA);
   }

   public void getPointOnB(Point3d pointOnBToPack)
   {
      pointOnBToPack.set(pointOnB);
   }
}
