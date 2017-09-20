package us.ihmc.exampleSimulations.controllerCore;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;

public interface ControllerCoreModeChangedListener
{
   void controllerCoreModeHasChanged(WholeBodyControllerCoreMode newMode);
}
