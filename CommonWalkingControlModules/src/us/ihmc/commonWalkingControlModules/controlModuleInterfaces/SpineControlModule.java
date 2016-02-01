package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.SpineTorques;

public interface SpineControlModule
{
   public abstract void doSpineControl(SpineTorques spineTorquesToPack);
}

