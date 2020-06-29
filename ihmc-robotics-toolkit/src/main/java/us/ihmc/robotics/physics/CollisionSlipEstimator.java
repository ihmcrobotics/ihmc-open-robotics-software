package us.ihmc.robotics.physics;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameUnitVector3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.collision.EuclidFrameShape3DCollisionResult;
import us.ihmc.robotics.EuclidCoreMissingTools;

public class CollisionSlipEstimator implements CollisionSlipHolder
{
   private final ReferenceFrame rootFrame;
   private Collidable collidableA, collidableB;

   private long numberOfTicksSinceLastContact = Long.MAX_VALUE;
   private long numberOfTicksToClear = 100;

   private final FramePoint3D pointOnA = new FramePoint3D();
   private final FramePoint3D pointOnB = new FramePoint3D();

   private final FrameVector3D separationFromBToA = new FrameVector3D();
   private final FrameUnitVector3D collisionAxis = new FrameUnitVector3D();
   private final FrameVector3D estimatedSlipFromBToA = new FrameVector3D();

   private final int estimatorID;

   public CollisionSlipEstimator(ReferenceFrame rootFrame, Collidable collidableA, Collidable collidableB)
   {
      this.rootFrame = rootFrame;
      this.collidableA = collidableA;
      this.collidableB = collidableB;
      estimatorID = PhysicsEngineTools.computeCollisionHashCode(collidableA, collidableB);
   }

   public boolean isEstimatorFor(Collidable candidateA, Collidable candidateB)
   {
      if (candidateA == collidableA)
         return candidateB == collidableB;
      else
         return candidateA == collidableB && candidateB == collidableA;
   }

   public void clear()
   {
      numberOfTicksSinceLastContact = Long.MAX_VALUE;
      estimatedSlipFromBToA.setToZero();
   }

   public void processPreCollisionDetection()
   {
      numberOfTicksSinceLastContact++;

      if (numberOfTicksSinceLastContact > 1)
      {
         if (numberOfTicksSinceLastContact > numberOfTicksToClear)
            clear();
         return;
      }

      /*
       * Beginning of the tick following when the collision was recorded. The 2 collidables have moved and
       * the reference frames of the collidables have been updated. When there is no slip, distance
       * orthogonal to the collision axis between pointOnA and pointOnB should be zero. We're measuring
       * this distance and accumulate it, that is our estimate of the slip.
       */
      pointOnA.changeFrame(rootFrame);
      pointOnB.changeFrame(rootFrame);

      separationFromBToA.sub(pointOnA, pointOnB);
      EuclidCoreMissingTools.extractTangentialPart(separationFromBToA, collisionAxis, separationFromBToA);
      estimatedSlipFromBToA.add(separationFromBToA);
   }

   public void processPostCollisiondetection(CollisionResult collisionResult)
   {
      if (collidableA != collisionResult.getCollidableA())
      {
         Collidable temp = collidableA;
         collidableA = collidableB;
         collidableB = temp;
      }

      if (collidableA != collisionResult.getCollidableA())
         throw new IllegalArgumentException("Wrong collidable");
      if (collidableB != collisionResult.getCollidableB())
         throw new IllegalArgumentException("Wrong collidable");

      EuclidFrameShape3DCollisionResult collisionData = collisionResult.getCollisionData();

      if (!collisionData.areShapesColliding())
         return;

      pointOnA.setIncludingFrame(collisionData.getPointOnA());
      pointOnB.setIncludingFrame(collisionData.getPointOnB());
      collisionAxis.set(collisionResult.getCollisionAxisForA());
      collisionResult.setCollisionSlipHolder(this);
      // Reset counter;
      numberOfTicksSinceLastContact = 0;
   }

   public void setNumberOfTicksToClear(long numberOfTicksToClear)
   {
      this.numberOfTicksToClear = numberOfTicksToClear;
   }

   public long getNumberOfTicksSinceLastContact()
   {
      return numberOfTicksSinceLastContact;
   }

   @Override
   public Collidable getCollidableA()
   {
      return collidableA;
   }

   @Override
   public Collidable getCollidableB()
   {
      return collidableB;
   }

   @Override
   public FrameVector3D getEstimatedSlipFromBToA()
   {
      return estimatedSlipFromBToA;
   }

   @Override
   public int hashCode()
   {
      return estimatorID;
   }

   @Override
   public String toString()
   {
      return "CollidableA: " + PhysicsEngineTools.collidableSimpleName(collidableA) + ", collidableB: " + PhysicsEngineTools.collidableSimpleName(collidableB)
            + ", slip: " + estimatedSlipFromBToA;
   }
}
