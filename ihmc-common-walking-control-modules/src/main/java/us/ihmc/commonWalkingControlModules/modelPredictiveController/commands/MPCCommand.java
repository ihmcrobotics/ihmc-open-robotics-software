package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import us.ihmc.euclid.interfaces.Settable;

/**
 * This represents the commands that are submitted to the MPC core.
 */
public interface MPCCommand<T extends MPCCommand<T>> extends Settable<T>
{
   /**
    * Retrieves the type of this command.
    * <p>
    * This is used to identify the type of the command so it can be handled properly inside the
    * mpc core.
    * </p>
    *
    * @return the type of this command.
    */
   MPCCommandType getCommandType();

   /**
    * Sets an id for this command, it will be passed over when copied in the the mpc core and
    * can be used to track elements that are using this command.
    *
    * @param id the new id for this command.
    */
   void setCommandId(int id);

   /**
    * Returns the id for this command.
    *
    * @return this command's id.
    */
   int getCommandId();
}
