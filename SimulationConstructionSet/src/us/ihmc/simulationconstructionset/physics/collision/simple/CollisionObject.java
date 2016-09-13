
package us.ihmc.simulationconstructionset.physics.collision.simple;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.simulationconstructionset.physics.CollisionShape;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;

public class CollisionObject implements CollisionShape
{
   private final CollisionShapeDescription description;
   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();

   public CollisionObject(CollisionShapeDescription collisionShape)
   {
      this.description = collisionShape;
   }

   @Override
   public boolean isGround()
   {
      // TODO Auto-generated method stub
      return false;
   }

   @Override
   public CollisionShapeDescription getDescription()
   {
      return description;
   }

   @Override
   public int getGroupMask()
   {
      return 0;
   }

   @Override
   public int getCollisionMask()
   {
      return 0;
   }

   @Override
   public void getTransformToWorld(RigidBodyTransform transformToWorldToPack)
   {
      transformToWorldToPack.set(transformToWorld);
   }

   @Override
   public void setTransformToWorld(RigidBodyTransform transformToWorld)
   {
      this.transformToWorld.set(transformToWorld);
   }

}
