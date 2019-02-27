package us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;

public interface VirtualModelControlCommand<T extends VirtualModelControlCommand<T>>
{
   void set(T other);

   ControllerCoreCommandType getCommandType();
}
