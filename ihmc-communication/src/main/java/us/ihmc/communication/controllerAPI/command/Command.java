package us.ihmc.communication.controllerAPI.command;

import org.apache.commons.lang3.NotImplementedException;

import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;

/**
 * A command has to be used when interacting with the {@link CommandInputManager} that represents
 * notably the walking controller input API.
 * <p>
 * As a message, a command holds onto the required data to execute for instance a body trajectory.
 * Unlike a message, a command HAS to be garbage free, and to be able to copy the data held in a
 * message through the {@link #setFromMessage(Settable)} method. When a new message is created and
 * one wants it to be consumed by the controller, the corresponding command has to be created and
 * registered to the input API.
 * </p>
 *
 * Any implementation of {@link Command} MUST have only ONE EMPTY constructor.
 *
 * @author Sylvain
 *
 * @param <C> Type of the final implementation of this command. It is used for the copy method
 *           {@link #setFromMessage(Settable)}.
 * @param <M> Type of the corresponding message that this command can copy.
 */
public interface Command<C extends Command<C, M>, M extends Settable<M>> extends Settable<C>
{
   /**
    * Clear all the data held in this Command. Used when the data has been processed by the controller
    * for instance. This HAS to be garbage free.
    */
   public abstract void clear();

   /**
    * Copy the data held in a message.
    * 
    * @param message message to copy the data from.
    */
   public abstract void setFromMessage(M message);

   /**
    * Informs which {@link Packet} class this Command is associated with.
    * 
    * @return {@link Packet} class this Command is associated with.
    */
   public abstract Class<M> getMessageClass();

   /**
    * This provides a quick check of the validity of the data held by this Command. Most common case is
    * to check if an enum is null or a trajectory is empty.
    * 
    * @return true if this command is valid.
    */
   public abstract boolean isCommandValid();

   /**
    * Gets the amount of time this command is to be delayed on the controller side before executing.
    * 
    * @return the time to delay this command in seconds.
    */
   public default double getExecutionDelayTime()
   {
      return 0.0;
   }

   /**
    * Sets the amount of time this command is to be delayed on the controller side before executing.
    * 
    * @param delayTime the time in seconds to delay after receiving the command before executing.
    */
   public default void setExecutionDelayTime(double delayTime)
   {
      throw new NotImplementedException(getClass().getSimpleName() + " does not implement setExecutionDelayTime");
   }

   /**
    * Informs the controller if this command supports delayed execution.
    */
   public default boolean isDelayedExecutionSupported()
   {
      return false;
   }

   /**
    * Returns the expected execution time of this command.
    * <p>
    * The execution time will be computed when the controller receives the command using the
    * controllers time plus the execution delay time. This is used when
    * {@code this.getExecutionDelayTime()} is non-zero.
    * </p>
    */
   public default double getExecutionTime()
   {
      return 0.0;
   }

   /**
    * Sets the execution time for this command. This is called by the controller when the command is
    * received.
    */
   public default void setExecutionTime(double adjustedExecutionTime)
   {
      throw new NotImplementedException(getClass().getSimpleName() + " does not implement setExecutionTime");
   }

   /**
    * Unique ID used to identify this message, should preferably be consecutively increasing.
    * <p>
    * The sequence ID can be used for instance when reporting the completion of a command, the output
    * is a status message that holds onto the sequence ID that it refers to.
    * </p>
    * 
    * @return the sequence ID.
    */
   long getSequenceId();
}
