package us.ihmc.commonWalkingControlModules.controllerCore.command.jacobianTranspose;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;

public interface JacobianTransposeCommand<T extends JacobianTransposeCommand<T>>
{
   public abstract void set(T other);

   public abstract ControllerCoreCommandType getCommandType();
}
