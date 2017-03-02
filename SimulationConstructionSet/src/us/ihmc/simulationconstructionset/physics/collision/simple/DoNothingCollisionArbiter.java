package us.ihmc.simulationconstructionset.physics.collision.simple;

import us.ihmc.simulationconstructionset.physics.CollisionArbiter;
import us.ihmc.simulationconstructionset.physics.collision.CollisionDetectionResult;

public class DoNothingCollisionArbiter implements CollisionArbiter
{
   private CollisionDetectionResult results;

   @Override
   public void processNewCollisions(CollisionDetectionResult newCollisions)
   {
      this.results = newCollisions;
   }

   @Override
   public CollisionDetectionResult getCollisions()
   {
      return results;
   }

}
