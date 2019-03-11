package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.euclid.interfaces.Settable;

public interface InverseKinematicsCommand<T extends InverseKinematicsCommand<T>> extends Settable<T>
{
   public abstract ControllerCoreCommandType getCommandType();
}
