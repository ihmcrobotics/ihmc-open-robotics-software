package us.ihmc.exampleSimulations.controllerCore.robotArmWithFixedBase;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;

public interface ControllerCoreModeChangedListener
{
   void controllerCoreModeHasChanged(WholeBodyControllerCoreMode newMode);
}
