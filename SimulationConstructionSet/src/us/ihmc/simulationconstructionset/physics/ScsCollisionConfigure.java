package us.ihmc.simulationconstructionset.physics;

import us.ihmc.simulationconstructionset.Robot;

/**
 * Configures the collision engine given the world model and collision interface
 *
 * @author Peter Abeles
 */
public interface ScsCollisionConfigure
{
   public void setup( Robot robot ,  ScsCollisionDetector collisionDetector );
}
