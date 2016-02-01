package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;

public interface PreSwingControlModule
{
   public abstract void doPreSwing(LegTorques legTorquesToPackForSwingLeg, double timeInPreSwingState);
}
