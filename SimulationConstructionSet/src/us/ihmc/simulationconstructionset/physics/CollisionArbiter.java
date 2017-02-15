package us.ihmc.simulationconstructionset.physics;

import us.ihmc.simulationconstructionset.physics.collision.CollisionDetectionResult;

public interface CollisionArbiter
{
   public abstract void processNewCollisions(CollisionDetectionResult newCollisions);
   public abstract CollisionDetectionResult getCollisions();
}
