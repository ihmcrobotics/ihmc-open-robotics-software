package us.ihmc.simulationconstructionset.physics;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.BoundingBox3d;

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
   public abstract void getBoundingBox(BoundingBox3d boundingBoxToPack);
   public abstract boolean isPointInside(Point3D pointInWorld);
   
   /**
    * Moves the given pointToRoll to where it would be if this shape was rolling over a surface with the given surfaceNormal.
    * @param surfaceNormal Surface normal of the surface this shape is rolling over.
    * @param pointToRoll Point to move to account for rolling.
    * @return true if the shape did roll. Else return false.
    */
   public abstract boolean rollContactIfRolling(Vector3D surfaceNormal, Point3D pointToRoll);
}
