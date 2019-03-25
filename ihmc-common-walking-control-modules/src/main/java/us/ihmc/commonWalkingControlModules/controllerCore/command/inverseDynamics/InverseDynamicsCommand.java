package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.euclid.interfaces.Settable;

public interface InverseDynamicsCommand<T extends InverseDynamicsCommand<T>> extends Settable<T>
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
   ControllerCoreCommandType getCommandType();
}
