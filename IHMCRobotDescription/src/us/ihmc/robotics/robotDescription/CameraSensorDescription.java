package us.ihmc.robotics.robotDescription;

import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;

public class CameraSensorDescription extends SensorDescription
{
   public CameraSensorDescription(String name, Vector3d offsetFromJoint)
   {
      super(name, offsetFromJoint);
   }

   public CameraSensorDescription(String name, RigidBodyTransform transformToJoint)
   {
      super(name, transformToJoint);
   }

}
