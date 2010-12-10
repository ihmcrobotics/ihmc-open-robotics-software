package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;

public interface HipDamperControlModule
{

   public abstract void addHipDamping(LegTorques legTorques);
   
}
