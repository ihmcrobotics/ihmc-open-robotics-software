package us.ihmc.simulationconstructionset.physics;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotDescription.CollisionMaskHolder;

/**
 * Object containing collision data for a specific instance of a shape.
 *
 */
public interface CollisionShape extends CollisionMaskHolder
{
   /**
    * Returns true if the shape is not mobile and part of the environment.
    */
   public abstract boolean isGround();
   
   /**
    * Set whether or not this collision shape is not mobile and part of the environment.
    * @param isGround
    */
   public abstract void setIsGround(boolean isGround);

   //   /**
   //    * Returns the distance a point is from the surface of the shape.  Positive values are outside and negative values are inside.
   //    */
   //   public double distance(double x, double y, double z);

   public abstract void getTransformToWorld(RigidBodyTransform transformToWorldToPack);

   public abstract void setTransformToWorld(RigidBodyTransform transformToWorld);

   public abstract void computeTransformedCollisionShape();

   /**
    * Returns a description of the shape.  Multiple {@link CollisionShape} can have the same description.
    */
   public abstract CollisionShapeDescription<?> getCollisionShapeDescription();

   public abstract CollisionShapeDescription<?> getTransformedCollisionShapeDescription();

}
