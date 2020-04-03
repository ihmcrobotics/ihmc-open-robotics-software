package us.ihmc.robotics.physics;

import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;

public interface MultiBodySystemStateReader
{
   void setMultiBodySystem(MultiBodySystemReadOnly multiBodySystem);

   void read();
}
