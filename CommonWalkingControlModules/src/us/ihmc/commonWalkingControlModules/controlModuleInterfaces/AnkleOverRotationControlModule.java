package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;

public interface AnkleOverRotationControlModule
{

   public abstract void resetForNextStep();

   public abstract void adjustLegTorquesToPreventOverRotation(LegTorques supportLegTorquesToPack);

}