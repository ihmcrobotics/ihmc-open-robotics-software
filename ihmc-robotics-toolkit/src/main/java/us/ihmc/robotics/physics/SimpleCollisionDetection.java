package us.ihmc.robotics.physics;

import java.util.Iterator;
import java.util.List;

import gnu.trove.map.TIntObjectMap;
import gnu.trove.map.hash.TIntObjectHashMap;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.collision.EuclidFrameShape3DCollisionResult;

public class SimpleCollisionDetection
{
   private final ReferenceFrame rootFrame;

   private final CollisionListResult allCollisions = new CollisionListResult();
   private final TIntObjectMap<CollisionListResult> previousCollisionMap = new TIntObjectHashMap<>();

   public SimpleCollisionDetection(ReferenceFrame rootFrame)
   {
      this.rootFrame = rootFrame;
   }

   public CollisionListResult evaluationCollisions(List<? extends CollidableHolder> dynamicCollidableHolders, CollidableHolder staticCollidableHolder)
   {
      allCollisions.clear();

      for (int i = 0; i < dynamicCollidableHolders.size(); i++)
      {
         CollidableHolder dynamicCollidableHolder = dynamicCollidableHolders.get(i);
         CollisionListResult collisionResults = new CollisionListResult();

         List<Collidable> dynamicCollidables = dynamicCollidableHolder.getCollidables();

         for (int j = 0; j < dynamicCollidables.size(); j++)
         {
            Collidable collidableA = dynamicCollidables.get(j);

            for (int k = j + 1; k < dynamicCollidables.size(); k++)
            {
               Collidable collidableB = dynamicCollidables.get(k);
               evaluateCollision(collidableA, collidableB, collisionResults);
            }
         }

         for (Collidable collidableRigidBody : dynamicCollidables)
         {
            for (Collidable staticCollidable : staticCollidableHolder)
            {
               evaluateCollision(collidableRigidBody, staticCollidable, collisionResults);
            }
         }

         for (int j = i + 1; j < dynamicCollidableHolders.size(); j++)
         {
            CollidableHolder otherDynamicCollidableHolder = dynamicCollidableHolders.get(j);

            for (Collidable collidableRigidBody : dynamicCollidables)
            {
               for (Collidable otherCollidableRigidBody : otherDynamicCollidableHolder.getCollidables())
               {
                  evaluateCollision(collidableRigidBody, otherCollidableRigidBody, collisionResults);
               }
            }
         }
      }

      return allCollisions;
   }

   private void evaluateCollision(Collidable collidableA, Collidable collidableB, CollisionListResult collisionListResultToPack)
   {
      if (!collidableA.isCollidableWith(collidableB))
         return;

      CollisionResult collision = pollCollision(collidableA, collidableB);
      collidableA.evaluateCollision(collidableB, collision);

      if (!collision.getCollisionData().areShapesColliding())
         return;

      boolean success = postCollisionDetection(collision);

      if (success)
      {
         allCollisions.add(collision);
         collisionListResultToPack.add(collision);
         registerCollision(collision);
      }
   }

   // Need to better handle numerical inaccuracies
   private boolean postCollisionDetection(CollisionResult collisionResult)
   {
      EuclidFrameShape3DCollisionResult collisionData = collisionResult.getCollisionData();
      FramePoint3D pointOnA = collisionData.getPointOnA();
      FramePoint3D pointOnB = collisionData.getPointOnB();
      FramePoint3D pointOnARootFrame = collisionResult.getPointOnARootFrame();
      FramePoint3D pointOnBRootFrame = collisionResult.getPointOnBRootFrame();
      pointOnARootFrame.setIncludingFrame(pointOnA);
      pointOnBRootFrame.setIncludingFrame(pointOnB);
      pointOnARootFrame.changeFrame(rootFrame);
      pointOnBRootFrame.changeFrame(rootFrame);

      FrameVector3D collisionAxis = collisionResult.getCollisionAxisForA();

      FrameVector3D normalOnA = collisionData.getNormalOnA();
      FrameVector3D normalOnB = collisionData.getNormalOnB();

      if (!normalOnA.containsNaN())
      {
         collisionAxis.setIncludingFrame(normalOnA);
         collisionAxis.negate();
      }
      else if (!normalOnB.containsNaN())
      {
         collisionAxis.setIncludingFrame(normalOnB);
      }
      else
      {
         collisionAxis.setReferenceFrame(rootFrame);
         collisionAxis.sub(pointOnBRootFrame, pointOnARootFrame);
      }

      double length = collisionAxis.length();
      if (length < 5.0e-5)
         return false;
      collisionAxis.scale(1.0 / length);

      if (collisionAxis.containsNaN())
         return false;

      collisionAxis.changeFrame(rootFrame);

      pointOnA.changeFrame(collisionData.getShapeA().getReferenceFrame());
      normalOnA.changeFrame(collisionData.getShapeA().getReferenceFrame());
      pointOnB.changeFrame(collisionData.getShapeB().getReferenceFrame());
      normalOnB.changeFrame(collisionData.getShapeB().getReferenceFrame());

      return true;
   }

   private void registerCollision(CollisionResult collision)
   {
      CollisionListResult previousCollisionList = previousCollisionMap.get(collision.hashCode());
      if (previousCollisionList == null)
      {
         previousCollisionList = new CollisionListResult();
         previousCollisionList.add(collision);
         previousCollisionMap.put(collision.hashCode(), previousCollisionList);
      }
      else
      {
         previousCollisionList.add(collision);
      }
   }

   private CollisionResult pollCollision(Collidable collidableA, Collidable collidableB)
   {
      int collisionID = CollisionResult.computeCollisionHashCode(collidableA, collidableB);
      CollisionListResult previousCollisionList = previousCollisionMap.get(collisionID);

      if (previousCollisionList == null)
         return newCollisionResult(collidableA, collidableB, collisionID);

      Iterator<CollisionResult> iterator = previousCollisionList.iterator();

      while (iterator.hasNext())
      {
         CollisionResult candidate = iterator.next();

         if (candidate.isCollisionOf(collidableA, collidableB))
         {
            iterator.remove();
            if (previousCollisionList.isEmpty())
               previousCollisionMap.remove(collisionID);

            if (candidate.getCollidableA() != collidableA)
               candidate.swapCollidables();

            preCollisionDetection(candidate);
            return candidate;
         }
      }

      return newCollisionResult(collidableA, collidableB, collisionID);
   }

   private void preCollisionDetection(CollisionResult previousCollision)
   {
      FrameVector3D collisionAxisForA = previousCollision.getCollisionAxisForA();
      if (collisionAxisForA == null)
         return;

      FramePoint3D currentPointA = new FramePoint3D(previousCollision.getCollisionData().getPointOnA());
      currentPointA.changeFrame(rootFrame);
      FramePoint3D previousPointA = previousCollision.getPointOnARootFrame();
      FrameVector3D pointADisplacement = new FrameVector3D(rootFrame);
      pointADisplacement.sub(currentPointA, previousPointA);
      double normalComponent = pointADisplacement.dot(collisionAxisForA);
      pointADisplacement.scaleAdd(-normalComponent, collisionAxisForA, pointADisplacement);

      FramePoint3D currentPointB = new FramePoint3D(previousCollision.getCollisionData().getPointOnB());
      currentPointB.changeFrame(rootFrame);
      FramePoint3D previousPointB = previousCollision.getPointOnBRootFrame();
      FrameVector3D pointBDisplacement = new FrameVector3D(rootFrame);
      pointBDisplacement.sub(currentPointB, previousPointB);
      normalComponent = pointBDisplacement.dot(collisionAxisForA);
      pointBDisplacement.scaleAdd(-normalComponent, collisionAxisForA, pointBDisplacement);

      FrameVector3D accumulatedSlip = previousCollision.getAccumulatedSlipForA();

      if (accumulatedSlip == null)
      {
         accumulatedSlip = new FrameVector3D(rootFrame);
         previousCollision.setAccumulatedSlipForA(accumulatedSlip);
      }

      accumulatedSlip.add(pointADisplacement);
      accumulatedSlip.sub(pointBDisplacement);
   }

   private CollisionResult newCollisionResult(Collidable collidableA, Collidable collidableB, int collisionID)
   {
      CollisionResult collisionResult = new CollisionResult();
      collisionResult.setCollidableA(collidableA);
      collisionResult.setCollidableB(collidableB);
      collisionResult.setCollisionID(collisionID);
      return collisionResult;
   }

   public CollisionListResult getAllCollisions()
   {
      return allCollisions;
   }
}
