package us.ihmc.robotics.physics;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.collision.EuclidFrameShape3DCollisionResult;
import us.ihmc.euclid.tools.EuclidHashCodeTools;

public class CollisionResult
{
   private final EuclidFrameShape3DCollisionResult collisionData = new EuclidFrameShape3DCollisionResult();

   private Collidable collidableA;
   private Collidable collidableB;

   private FramePoint3D pointOnARootFrame = new FramePoint3D();
   private FramePoint3D pointOnBRootFrame = new FramePoint3D();

   private FrameVector3D accumulatedSlipForA;

   /**
    * The collision direction (normalized) for the shape A:
    *
    * <pre>
    *  collisionAxisForA &equiv; normalOnB
    * </pre>
    */
   private final FrameVector3D collisionAxisForA = new FrameVector3D();

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
      collisionID = computeCollisionHashCode(collidableA, collidableB);
   }

   public void setCollidableA(Collidable collidableA)
   {
      this.collidableA = collidableA;
   }

   public void setCollidableB(Collidable collidableB)
   {
      this.collidableB = collidableB;
   }

   public void setAccumulatedSlipForA(FrameVector3D accumulatedSlipForA)
   {
      this.accumulatedSlipForA = accumulatedSlipForA;
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
    *  collisionAxisForA &equiv; pointOnA - pointOnB
    * </pre>
    */
   public FrameVector3D getCollisionAxisForA()
   {
      return collisionAxisForA;
   }

   public FrameVector3D getAccumulatedSlipForA()
   {
      return accumulatedSlipForA;
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
      return prefix + ", collidableA: " + collidableSimpleName(collidableA) + ", collidableB: " + collidableSimpleName(collidableB);
   }

   private String collidableSimpleName(Collidable collidable)
   {
      return collidable.getRigidBody() != null ? collidable.getRigidBody().getName() : "static";
   }

   public static int computeCollisionHashCode(Collidable collidableA, Collidable collidableB)
   {
      int collidableAID = collidableA.hashCode();
      int collidableBID = collidableB.hashCode();
      if (collidableAID > collidableBID)
         return EuclidHashCodeTools.toIntHashCode(EuclidHashCodeTools.combineHashCode(collidableAID, collidableBID));
      else
         return EuclidHashCodeTools.toIntHashCode(EuclidHashCodeTools.combineHashCode(collidableBID, collidableAID));
   }
}
