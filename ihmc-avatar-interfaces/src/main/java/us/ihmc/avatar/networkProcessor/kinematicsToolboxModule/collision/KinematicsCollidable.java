package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.collision;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

public class KinematicsCollidable
{
   private final RigidBodyBasics rigidBody;
   private final int collisionMask;
   private final int collisionGroup;
   private final Shape3DReadOnly shape;
   private final ReferenceFrame shapeFrame;
   private final double minimumSafeDistance;

   public KinematicsCollidable(RigidBodyBasics rigidBody, int collisionMask, int collisionGroup, Shape3DReadOnly shape, ReferenceFrame shapeFrame, double minimumSafeDistance)
   {
      this.rigidBody = rigidBody;
      this.collisionMask = collisionMask;
      this.collisionGroup = collisionGroup;
      this.shape = shape;
      this.shapeFrame = shapeFrame;
      this.minimumSafeDistance = minimumSafeDistance;
   }

   public boolean isCollidableWith(KinematicsCollidable other)
   {
      if ((collisionGroup & other.collisionMask) == 0x00)
         return false;
      if ((other.collisionGroup & collisionMask) == 0x00)
         return false;
      return true;
   }

   public KinematicsCollisionResult evaluateCollision(KinematicsCollidable other)
   {
      return KinematicsCollisionTools.evaluateShape3DShape3DCollision(shape, shapeFrame, other.shape, other.shapeFrame);
   }

   public RigidBodyBasics getRigidBody()
   {
      return rigidBody;
   }

   public int getCollisionMask()
   {
      return collisionMask;
   }

   public int getCollisionGroup()
   {
      return collisionGroup;
   }

   public Shape3DReadOnly getShape()
   {
      return shape;
   }

   public double getMinimumSafeDistance()
   {
      return minimumSafeDistance;
   }
}
