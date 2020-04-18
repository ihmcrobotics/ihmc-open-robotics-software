package us.ihmc.robotics.physics;

import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;

public interface ExternalWrenchReader
{
   void readExternalWrench(RigidBodyReadOnly rigidBody, WrenchReadOnly externalWrench);
}
