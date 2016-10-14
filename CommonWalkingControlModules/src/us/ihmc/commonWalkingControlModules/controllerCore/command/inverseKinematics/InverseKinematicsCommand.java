package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;

public interface InverseKinematicsCommand<T extends InverseKinematicsCommand<T>>
{
   public abstract void set(T other);

   public abstract ControllerCoreCommandType getCommandType();
}
