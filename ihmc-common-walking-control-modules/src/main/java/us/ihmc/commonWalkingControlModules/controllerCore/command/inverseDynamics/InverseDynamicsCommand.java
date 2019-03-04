package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.euclid.interfaces.Settable;

public interface InverseDynamicsCommand<T extends InverseDynamicsCommand<T>> extends Settable<T>
{
   public abstract ControllerCoreCommandType getCommandType();
}
