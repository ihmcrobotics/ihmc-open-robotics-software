package us.ihmc.simulationconstructionset.physics;

import us.ihmc.simulationconstructionset.physics.visualize.DefaultCollisionVisualizer;

/**
 * Data structure which contains references to high level implementations of the physics code
 *
 */
public class ScsPhysics
{
   // TODO having collisionConfigure here is kinda ugly.  it is robot specific so it is created elsewhere from everything else, but still is needed
   //      at the same time by the simulation
   public ScsCollisionConfigure collisionConfigure;
   public ScsCollisionDetector collisionDetector;
   public CollisionArbiter collisionArbiter;
   public CollisionHandler collisionHandler;
   public DefaultCollisionVisualizer visualize;

   public ScsPhysics(ScsCollisionConfigure collisionConfigure,
                     ScsCollisionDetector collisionDetector,
                     CollisionArbiter collisionArbiter,
                     CollisionHandler collisionHandler,
                     DefaultCollisionVisualizer visualize )
   {
      this.collisionConfigure = collisionConfigure;
      this.collisionDetector = collisionDetector;
      this.collisionArbiter = collisionArbiter;
      this.collisionHandler = collisionHandler;
      this.visualize = visualize;
   }
}
