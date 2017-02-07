
package us.ihmc.simulationconstructionset.physics.collision.simple;

import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.simulationconstructionset.physics.CollisionShape;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;

public class SimpleCollisionShape implements CollisionShape
{
   private final CollisionShapeDescription collisionShapeDescription;
   private final CollisionShapeDescription transformedCollisionShapeDescription;
   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();

   private int groupMask = 0x00;
   private int collisionMask = 0x00;
   
   private boolean isGround = false;

   public SimpleCollisionShape(CollisionShapeDescription collisionShapeDescription)
   {
      this.collisionShapeDescription = collisionShapeDescription;
      this.transformedCollisionShapeDescription = collisionShapeDescription.copy();
   }

   @Override
   public boolean isGround()
   {
      return isGround;
   }
   
   @Override
   public void setIsGround(boolean isGround)
   {
      this.isGround = isGround;
   }

   @Override
   public CollisionShapeDescription getCollisionShapeDescription()
   {
      return collisionShapeDescription;
   }

   @Override
   public int getCollisionGroup()
   {
      return groupMask;
   }

   @Override
   public int getCollisionMask()
   {
      return collisionMask;
   }
   
   @Override
   public void setCollisionGroup(int groupMask)
   {
      this.groupMask = groupMask; 
   }

   @Override
   public void setCollisionMask(int collisionMask)
   {
      this.collisionMask = collisionMask;
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

   @Override
   public CollisionShapeDescription getTransformedCollisionShapeDescription()
   {
      return transformedCollisionShapeDescription;
   }

   @Override
   public void computeTransformedCollisionShape()
   {
      transformedCollisionShapeDescription.setFrom(collisionShapeDescription);
      this.getTransformToWorld(transformToWorld);
      transformedCollisionShapeDescription.applyTransform(transformToWorld);
   }

   @Override
   public BoundingBox3d getBoundingBox()
   {
      return transformedCollisionShapeDescription.getBoundingBox();
   }

}
