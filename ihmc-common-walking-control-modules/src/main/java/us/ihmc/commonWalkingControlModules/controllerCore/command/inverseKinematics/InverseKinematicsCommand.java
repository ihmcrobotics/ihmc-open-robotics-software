package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.euclid.interfaces.Settable;

public interface InverseKinematicsCommand<T extends InverseKinematicsCommand<T>> extends Settable<T>
{
   /**
    * Retrieves the type of this command.
    * <p>
    * This is used to identify the type of the command so it can be handled properly inside the
    * controller core.
    * </p>
    * 
    * @return the type of this command.
    */
   public abstract ControllerCoreCommandType getCommandType();
}
