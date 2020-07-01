package us.ihmc.robotics.physics;

import gnu.trove.iterator.TIntObjectIterator;
import gnu.trove.map.TIntObjectMap;
import gnu.trove.map.hash.TIntObjectHashMap;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public class CollisionSlipEstimatorMap
{
   private final TIntObjectMap<CollisionSlipEstimator> estimatorMap = new TIntObjectHashMap<>();
   private final ReferenceFrame rootFrame;

   public CollisionSlipEstimatorMap(ReferenceFrame rootFrame)
   {
      this.rootFrame = rootFrame;
   }

   public void processPreCollisionDetection()
   {
      for (TIntObjectIterator<CollisionSlipEstimator> iterator = estimatorMap.iterator(); iterator.hasNext();)
      {
         iterator.advance();
         iterator.value().processPreCollisionDetection();
      }
   }

   public void processPostCollisionDetection(CollisionListResult allCollisions)
   {
      for (CollisionResult collision : allCollisions)
      {
         CollisionSlipEstimator estimator = estimatorMap.get(collision.hashCode());

         if (estimator == null)
         {
            estimator = new CollisionSlipEstimator(rootFrame, collision.getCollidableA(), collision.getCollidableB());
            estimatorMap.put(collision.hashCode(), estimator);
         }

         estimator.processPostCollisiondetection(collision);
      }
   }
}
