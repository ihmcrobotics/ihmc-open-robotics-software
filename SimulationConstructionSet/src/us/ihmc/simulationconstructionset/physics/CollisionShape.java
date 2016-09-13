package us.ihmc.simulationconstructionset.physics;

import us.ihmc.robotics.geometry.RigidBodyTransform;

/**
 * Object containing collision data for a specific instance of a shape.
 *
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
    * Bit field indicating which groups the shape belongs to
    */
   public int getGroupMask();

   /**
    * Bit field indicating which groups the shape can collide against
    */
   public int getCollisionMask();

//   /**
//    * Returns the distance a point is from the surface of the shape.  Positive values are outside and negative values are inside.
//    */
//   public double distance(double x, double y, double z);

   public void getTransformToWorld(RigidBodyTransform transformToWorldToPack);

   public void setTransformToWorld(RigidBodyTransform transformToWorld);
}
