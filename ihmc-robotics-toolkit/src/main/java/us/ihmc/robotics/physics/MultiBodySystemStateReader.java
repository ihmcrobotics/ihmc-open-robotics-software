package us.ihmc.robotics.physics;

import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;

public interface MultiBodySystemStateReader
{
   void setMultiBodySystem(MultiBodySystemReadOnly multiBodySystem);

   void read();

   static RigidBodyReadOnly findRigidBody(String name, MultiBodySystemReadOnly multiBodySystem)
   {
      return multiBodySystem.getRootBody().subtreeStream().filter(body -> body.getName().equals(name)).findAny().orElse(null);
   }

   static JointReadOnly findJoint(String name, MultiBodySystemReadOnly multiBodySystem)
   {
      return multiBodySystem.getAllJoints().stream().filter(joint -> joint.getName().equals(name)).findAny().orElse(null);
   }
}
