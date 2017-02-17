package us.ihmc.robotics.robotDescription;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;

public class JointWrenchSensorDescription extends SensorDescription
{

   public JointWrenchSensorDescription(String name, Vector3D offsetFromJoint)
   {
      super(name, offsetFromJoint);
   }

   public JointWrenchSensorDescription(String name, RigidBodyTransform transformToJoint)
   {
      super(name, transformToJoint);
   }

}
