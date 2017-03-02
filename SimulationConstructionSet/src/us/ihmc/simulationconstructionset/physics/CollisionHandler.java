package us.ihmc.simulationconstructionset.physics;

import java.util.ArrayList;

import us.ihmc.simulationconstructionset.ContactingExternalForcePoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.physics.collision.CollisionDetectionResult;
import us.ihmc.simulationconstructionset.physics.collision.CollisionHandlerListener;

/**
 * Interface for the physics engine respond to a collision.  Collisions are modeled as a set of point collisions.
 *
 */
public interface CollisionHandler
{

   /**
    * Invoked each simulation cycle before collision detection has started.
    */
   public abstract void maintenanceBeforeCollisionDetection();

   /**
    * Invoked each simulation cycle after collision detection has finished.
    */
   public abstract void maintenanceAfterCollisionDetection();

   /**
    * Adds a new collision listener
    *
    * @param listener The listener which is to be added
    */
   public abstract void addListener(CollisionHandlerListener listener);

   /**
    * When two shapes collide this function is called.  The two shapes and which points on the shapes collide are passed in.
    *
    * @param shapeA One of the shapes which is colliding
    * @param shapeB One of the shapes which is colliding
    * @param contacts Which points on the two shapes are involved in the collision
    */
   public abstract void handle(Contacts contacts);

   public abstract void handleCollisions(CollisionDetectionResult results);

   public abstract void addContactingExternalForcePoints(Link link, ArrayList<ContactingExternalForcePoint> contactingExternalForcePoints);

}
