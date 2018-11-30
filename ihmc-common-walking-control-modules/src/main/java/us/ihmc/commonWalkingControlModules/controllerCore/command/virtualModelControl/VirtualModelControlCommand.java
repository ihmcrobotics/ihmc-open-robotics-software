package us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;

public interface VirtualModelControlCommand<T extends VirtualModelControlCommand>
{
   void set(T other);

   ControllerCoreCommandType getCommandType();
}
