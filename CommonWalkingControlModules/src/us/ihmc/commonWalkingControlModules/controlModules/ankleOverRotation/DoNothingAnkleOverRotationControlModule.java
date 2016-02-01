package us.ihmc.commonWalkingControlModules.controlModules.ankleOverRotation;

import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.AnkleOverRotationControlModule;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;

public class DoNothingAnkleOverRotationControlModule implements AnkleOverRotationControlModule
{

   public void adjustLegTorquesToPreventOverRotation(LegTorques supportLegTorquesToPack)
   {
      // Do nothing.
   }

   public void resetForNextStep()
   {   
      // Do nothing.
   }

}
