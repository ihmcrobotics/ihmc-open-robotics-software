package us.ihmc.robotics.physics;

import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;

public interface MultiBodySystemStateWriter
{
   void setMultiBodySystem(MultiBodySystemBasics multiBodySystem);

   void write();
}
