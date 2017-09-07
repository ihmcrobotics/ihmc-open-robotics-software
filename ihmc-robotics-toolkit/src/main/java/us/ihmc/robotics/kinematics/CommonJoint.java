package us.ihmc.robotics.kinematics;

import us.ihmc.euclid.transform.RigidBodyTransform;

public interface CommonJoint
{
   public abstract RigidBodyTransform getOffsetTransform3D();
   public abstract RigidBodyTransform getJointTransform3D();
}
