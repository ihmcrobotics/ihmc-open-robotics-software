package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.collision;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

public class KinematicsCollidable
{
   private final RigidBodyBasics rigidBody;
   private final int groupId;
   private final Shape3DReadOnly shape;
   private final ReferenceFrame shapeFrame;

   public KinematicsCollidable(RigidBodyBasics rigidBody, int groupId, Shape3DReadOnly shape, ReferenceFrame shapeFrame)
   {
      this.rigidBody = rigidBody;
      this.groupId = groupId;
      this.shape = shape;
      this.shapeFrame = shapeFrame;
   }

   public boolean isCollidableWith(KinematicsCollidable other)
   {
      return groupId != other.groupId;
   }

   public KinematicsCollisionResult evaluateCollision(KinematicsCollidable other)
   {
      return KinematicsCollisionTools.evaluateShape3DShape3DCollision(shape, shapeFrame, other.shape, other.shapeFrame);
   }

   public RigidBodyBasics getRigidBody()
   {
      return rigidBody;
   }

   public int getGroupId()
   {
      return groupId;
   }

   public Shape3DReadOnly getShape()
   {
      return shape;
   }
}
