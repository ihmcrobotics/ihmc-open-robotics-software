package us.ihmc.simulationconstructionset.physics;

import us.ihmc.simulationconstructionset.physics.collision.CollisionDetectionResult;

/**
 * High level interface for collision detection
 *
 */
public interface ScsCollisionDetector
{
   /**
    * Call to initialize collision detection.
    *
    */
   public void initialize();

   /**
    * Returns a factory for creating collision shapes that are attached to Links.
    */
   public CollisionShapeFactory getShapeFactory();

   /**
    * Checks for collisions. Puts the results of the collision detection process into result
    */
   public void performCollisionDetection(CollisionDetectionResult result);

}
