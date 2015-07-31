package us.ihmc.simulationconstructionset.physics;

import us.ihmc.simulationconstructionset.Link;
import us.ihmc.robotics.geometry.RigidBodyTransform;

/**
 * Object containing collision data for a specific instance of a shape.
 *
 * @author Peter Abeles
 */
public interface CollisionShape
{
   /**
    * Returns true if the shape is not mobile and part of the environment.
    */
   public boolean isGround();
   
   /**
    * Returns a description of the shape.  Multiple {@link CollisionShape} can have the same description.
    */
   public CollisionShapeDescription getDescription();

   /**
    * The {@link Link} which this shape is attached to.
    */
   public Link getLink();

   /**
    * Transform from shape to link coordinates.
    */
   public RigidBodyTransform getShapeToLink();

   /**
    * Bit field indicating which groups the shape belongs to
    */
   public int getGroupMask();

   /**
    * Bit field indicating which groups the shape can collide against
    */
   public int getCollisionMask();

   /**
    * Returns the distance a point is from the surface of the shape.  Positive values are outside and negative values are inside.
    */
   public double distance(double x, double y, double z);
}
