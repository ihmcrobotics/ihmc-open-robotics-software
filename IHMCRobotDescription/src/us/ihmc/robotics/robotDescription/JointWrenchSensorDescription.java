package us.ihmc.robotics.robotDescription;

import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;

public class JointWrenchSensorDescription extends SensorDescription
{

   public JointWrenchSensorDescription(String name, Vector3d offsetFromJoint)
   {
      super(name, offsetFromJoint);
   }

   public JointWrenchSensorDescription(String name, RigidBodyTransform transformToJoint)
   {
      super(name, transformToJoint);
   }

}
