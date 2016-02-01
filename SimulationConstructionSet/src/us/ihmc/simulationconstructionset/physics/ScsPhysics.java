package us.ihmc.simulationconstructionset.physics;

import us.ihmc.simulationconstructionset.physics.visualize.DefaultCollisionVisualize;

/**
 * Data structure which contains references to high level implementations of the physics code
 *
 * @author Peter Abeles
 */
public class ScsPhysics
{
   // TODO having collisionConfigure here is kinda ugly.  it is robot specific so it is created elsewhere from everything else, but still is needed
   //      at the same time by the simulation
   public ScsCollisionConfigure collisionConfigure;
   public ScsCollisionDetector collisionDetector;
   public DefaultCollisionVisualize visualize;

   public ScsPhysics(ScsCollisionConfigure collisionConfigure,
                     ScsCollisionDetector collisionDetector,
                     DefaultCollisionVisualize visualize )
   {
      this.collisionConfigure = collisionConfigure;
      this.collisionDetector = collisionDetector;
      this.visualize = visualize;
   }
}
