package us.ihmc.simulationconstructionset.physics;

import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.ExternalTorque;

/**
 * Interface for the physics engine respond to a collision.  Collisions are modeled as a set of point collisions.
 *
 * @author Peter Abeles
 */
public interface CollisionHandler
{
   /**
    * Implementation should do any initialization in here that must be done before it can start.
    * @param collision Reference to collision detector
    */
   public void initialize(ScsCollisionDetector collision);

   /**
    * Invoked each simulation cycle before collision detection has started.
    */
   public void maintenanceBeforeCollisionDetection();

   /**
    * Invoked each simulation cycle after collision detection has finished.
    */
   public void maintenanceAfterCollisionDetection();
   
   /**
    * Adds a new collision listener
    *
    * @param listener The listener which is to be added
    */
   public void addListener(Listener listener);

   /**
    * When two shapes collide this function is called.  The two shapes and which points on the shapes collide are passed in.
    *
    * @param shapeA One of the shapes which is colliding
    * @param shapeB One of the shapes which is colliding
    * @param contacts Which points on the two shapes are involved in the collision
    */
   public void handle(CollisionShape shapeA, CollisionShape shapeB, Contacts contacts);

   /**
    * Called whenever a collision happens
    */
   public static interface Listener
   {
      public void collision(CollisionShape shapeA, CollisionShape shapeB, ExternalForcePoint forceA, ExternalForcePoint forceB, ExternalTorque torqueA,
                            ExternalTorque torqueB);
   }
}
