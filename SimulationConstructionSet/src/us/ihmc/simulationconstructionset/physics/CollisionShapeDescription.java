package us.ihmc.simulationconstructionset.physics;

import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.robotics.geometry.RigidBodyTransform;

/**
 * Description of the collision shape.  The same description can be linked to multiple objects ({@link us.ihmc.simulationconstructionset.Link})
 * to conserve memory.  For example, you could create a description for a standard brick then add 10000 of them to the scene.
 *
 */
public interface CollisionShapeDescription<T extends CollisionShapeDescription<T>>
{
   public abstract void setFrom(T collisionShapeDescription);
   public abstract void applyTransform(RigidBodyTransform transformToWorld);
   public abstract CollisionShapeDescription<T> copy();
   public abstract BoundingBox3d getBoundingBox();
}
