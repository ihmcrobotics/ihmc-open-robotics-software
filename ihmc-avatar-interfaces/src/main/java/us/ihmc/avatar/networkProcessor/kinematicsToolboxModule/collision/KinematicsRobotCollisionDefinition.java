package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.collision;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;

public interface KinematicsRobotCollisionDefinition
{
   double getMinimumSafeDistance();

   int getCollisionGroup(RigidBodyReadOnly rigidBody);

   Shape3DReadOnly getCollisionShape(RigidBodyReadOnly rigidBody);

   ReferenceFrame getCollisionShapeFrame(RigidBodyReadOnly rigidBody);
}
