package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;

public interface InverseDynamicsCommand<T extends InverseDynamicsCommand<T>>
{
   public abstract void set(T other);

   public abstract ControllerCoreCommandType getCommandType();
}
