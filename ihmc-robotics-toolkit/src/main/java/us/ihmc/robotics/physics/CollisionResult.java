package us.ihmc.robotics.physics;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameUnitVector3D;
import us.ihmc.euclid.referenceFrame.collision.EuclidFrameShape3DCollisionResult;

public class CollisionResult
{
   private final EuclidFrameShape3DCollisionResult collisionData = new EuclidFrameShape3DCollisionResult();

   private Collidable collidableA;
   private Collidable collidableB;

   private final FramePoint3D pointOnARootFrame = new FramePoint3D();
   private final FramePoint3D pointOnBRootFrame = new FramePoint3D();

   /**
    * The collision direction (normalized) for the shape A:
    *
    * <pre>
    *  collisionAxisForA &equiv; normalOnB
    * </pre>
    */
   private final FrameUnitVector3D collisionAxisForA = new FrameUnitVector3D();

   private int collisionID = 0;

   /**
    * Creates a new empty collision result.
    */
   public CollisionResult()
   {
      collisionAxisForA.setToNaN();
   }

   public boolean isCollisionOf(Collidable candidateA, Collidable candidateB)
   {
      if (candidateA == collidableA)
         return candidateB == collidableB;
      else
         return candidateA == collidableB && candidateB == collidableA;
   }

   public void setCollisionID(int collisionID)
   {
      this.collisionID = collisionID;
   }

   public void updateCollisionID()
   {
      collisionID = PhysicsEngineTools.computeCollisionHashCode(collidableA, collidableB);
   }

   public void setCollidableA(Collidable collidableA)
   {
      this.collidableA = collidableA;
   }

   public void setCollidableB(Collidable collidableB)
   {
      this.collidableB = collidableB;
   }

   public void swapCollidables()
   {
      collisionData.swapShapes();
      collisionAxisForA.negate();
   }

   public EuclidFrameShape3DCollisionResult getCollisionData()
   {
      return collisionData;
   }

   public Collidable getCollidableA()
   {
      return collidableA;
   }

   public Collidable getCollidableB()
   {
      return collidableB;
   }

   public FramePoint3D getPointOnARootFrame()
   {
      return pointOnARootFrame;
   }

   public FramePoint3D getPointOnBRootFrame()
   {
      return pointOnBRootFrame;
   }

   /**
    * The collision direction (normalized) for the shape A:
    *
    * <pre>
    *  collisionAxisForA &equiv; normalOnB
    *  collisionAxisForA &equiv; -normalOnA
    *  collisionAxisForA &equiv; (pointOnA - pointOnB)/|pointOnA - pointOnB|
    * </pre>
    */
   public FrameUnitVector3D getCollisionAxisForA()
   {
      return collisionAxisForA;
   }

   @Override
   public int hashCode()
   {
      if (collisionID == 0L)
         updateCollisionID();
      return collisionID;
   }

   @Override
   public String toString()
   {
      String prefix = collisionData.areShapesColliding() ? "Colliding" : "Non-colliding";
      return prefix + ", collidableA: " + PhysicsEngineTools.collidableSimpleName(collidableA) + ", collidableB: "
            + PhysicsEngineTools.collidableSimpleName(collidableB);
   }
}
