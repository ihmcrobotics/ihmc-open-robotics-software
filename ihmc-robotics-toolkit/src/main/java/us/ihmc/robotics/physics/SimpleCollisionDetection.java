package us.ihmc.robotics.physics;

import java.util.Iterator;
import java.util.List;

import gnu.trove.map.TIntObjectMap;
import gnu.trove.map.hash.TIntObjectHashMap;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameUnitVector3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.collision.EuclidFrameShape3DCollisionResult;

public class SimpleCollisionDetection
{
   private final ReferenceFrame rootFrame;

   private double minimumPenetration = 5.0e-5;

   private final CollisionListResult allCollisions = new CollisionListResult();
   private final TIntObjectMap<CollisionListResult> previousCollisionMap = new TIntObjectHashMap<>();

   public SimpleCollisionDetection(ReferenceFrame rootFrame)
   {
      this.rootFrame = rootFrame;
   }

   public void setMinimumPenetration(double minimumPenetration)
   {
      this.minimumPenetration = minimumPenetration;
   }

   public CollisionListResult evaluationCollisions(List<? extends CollidableHolder> dynamicCollidableHolders, CollidableHolder staticCollidableHolder, double dt)
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
               evaluateCollision(collidableA, collidableB, collisionResults, dt);
            }
         }

         for (Collidable collidableRigidBody : dynamicCollidables)
         {
            for (Collidable staticCollidable : staticCollidableHolder)
            {
               evaluateCollision(collidableRigidBody, staticCollidable, collisionResults, dt);
            }
         }

         for (int j = i + 1; j < dynamicCollidableHolders.size(); j++)
         {
            CollidableHolder otherDynamicCollidableHolder = dynamicCollidableHolders.get(j);

            for (Collidable collidableRigidBody : dynamicCollidables)
            {
               for (Collidable otherCollidableRigidBody : otherDynamicCollidableHolder.getCollidables())
               {
                  evaluateCollision(collidableRigidBody, otherCollidableRigidBody, collisionResults, dt);
               }
            }
         }
      }

      return allCollisions;
   }

   private void evaluateCollision(Collidable collidableA, Collidable collidableB, CollisionListResult collisionListResultToPack, double dt)
   {
      if (!collidableA.isCollidableWith(collidableB))
         return;

      CollisionResult collision = pollCollision(collidableA, collidableB);
      collidableA.evaluateCollision(dt, collidableB, collision);

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

      if (collisionData.getDistance() < minimumPenetration)
         return false;

      FramePoint3D pointOnA = collisionData.getPointOnA();
      FramePoint3D pointOnB = collisionData.getPointOnB();
      FramePoint3D pointOnARootFrame = collisionResult.getPointOnARootFrame();
      FramePoint3D pointOnBRootFrame = collisionResult.getPointOnBRootFrame();
      pointOnARootFrame.setIncludingFrame(pointOnA);
      pointOnBRootFrame.setIncludingFrame(pointOnB);
      pointOnARootFrame.changeFrame(rootFrame);
      pointOnBRootFrame.changeFrame(rootFrame);

      FrameUnitVector3D collisionAxis = collisionResult.getCollisionAxisForA();

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
      int collisionID = PhysicsEngineTools.computeCollisionHashCode(collidableA, collidableB);
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

            return candidate;
         }
      }

      return newCollisionResult(collidableA, collidableB, collisionID);
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
