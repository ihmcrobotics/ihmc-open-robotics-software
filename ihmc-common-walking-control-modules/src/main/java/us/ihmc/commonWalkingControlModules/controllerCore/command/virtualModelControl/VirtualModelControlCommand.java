package us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.euclid.interfaces.Settable;

public interface VirtualModelControlCommand<T extends VirtualModelControlCommand<T>> extends Settable<T>
{
   ControllerCoreCommandType getCommandType();
}
