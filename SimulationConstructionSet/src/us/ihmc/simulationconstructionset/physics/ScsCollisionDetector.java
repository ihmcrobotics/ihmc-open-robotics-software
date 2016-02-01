package us.ihmc.simulationconstructionset.physics;

import us.ihmc.simulationconstructionset.Link;

/**
 * High level interface for collision detection
 *
 * @author Peter Abeles
 */
public interface ScsCollisionDetector
{
   /**
    * Call to initialize collision detection.
    *
    * @param handler Function used to handle collisions
    */
   public void initialize(CollisionHandler handler);

   /**
    * Returns a factory for creating collision shapes that are attached to Links.
    */
   public CollisionShapeFactory getShapeFactory();

   /**
    * Removes a collision shape
    */
   public void removeShape(Link link);

   /**
    * Returns the collision shape associated with the link.  Worst case this can be O(n)
    */
   public CollisionShape lookupCollisionShape(Link link);

   /**
    * Call after each simulation step to check for collisions. This function must call
    * {@link us.ihmc.simulationconstructionset.physics.CollisionHandler#maintenance()}
    * after all collision detection has been performed
    */
   public void performCollisionDetection();
}
