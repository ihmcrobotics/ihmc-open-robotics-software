package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.SpineTorques;

public interface SpineLungingControlModule extends SpineControlModule
{
   public abstract void doMaintainDesiredChestOrientation();
   
   public abstract void getSpineTorques(SpineTorques spineTorquesToPack);

}

