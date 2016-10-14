package us.ihmc.simulationconstructionset.physics;

import us.ihmc.simulationconstructionset.Robot;

/**
 * Configures the collision engine given the world model and collision interface
 *
 */
public interface ScsCollisionConfigure
{
   public void setup(Robot robot,  ScsCollisionDetector collisionDetector, CollisionHandler collisionHandler);
}
