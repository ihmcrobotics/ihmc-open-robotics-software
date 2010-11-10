package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.ArmTorques;


public interface ArmControlModule
{
   public abstract void doArmControl(ArmTorques[] armTorquesToPack);
}
