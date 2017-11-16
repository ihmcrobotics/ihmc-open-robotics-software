package us.ihmc.communication.controllerAPI.command;

import org.apache.commons.lang3.NotImplementedException;

import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.lists.Settable;

/**
 * A Command has to be used when interacting with the {@link CommandInputManager} that represents notably the walking controller input API.
 * As a {@link Packet}, a Command holds onto the required data execute a body trajectory for instance.
 * Unlike a {@link Packet}, a Command HAS to be garbage free, and to be able to copy the data held in a message through the {@link #set(Packet)} method.
 * When a new {@link Packet} is created and one wants it to be consumed by the controller, the corresponding Command has to be created and registered to the input API.
 *
 * Any implementation of {@link Command} MUST have only ONE EMPTY constructor.
 *
 * @author Sylvain
 *
 * @param <C> Type of the final implementation of this Command. It is used for the copy method {@link #set(M)}.
 * @param <M> Type of the corresponding {@link Packet} that this Command can copy.
 */
public interface Command<C extends Command<C, M>, M extends Packet<M>> extends Settable<C>
{
   /**
    * Clear all the data held in this Command.
    * Used when the data has been processed by the controller for instance.
    * This HAS to be garbage free.
    */
   public abstract void clear();

   /**
    * Copy the data held in a message.
    * @param message message to copy the data from.
    */
   public abstract void set(M message);

   /**
    * Informs which {@link Packet} class this Command is associated with.
    * @return {@link Packet} class this Command is associated with.
    */
   public abstract Class<M> getMessageClass();

   /**
    * This provides a quick check the validity of the data held by this Command.
    * Most common case is to check a enum is null or a trajectory is empty.
    * @return true if this Command is valid.
    */
   public abstract boolean isCommandValid();
   
   /**
    * returns the amount of time this command is delayed on the controller side before executing
    * @return the time to delay this command in seconds
    */
   public default double getExecutionDelayTime()
   {
      return 0.0;
   }
   
   /**
    * sets the amount of time this command is delayed on the controller side before executing
    * @param delayTime the time in seconds to delay after receiving the command before executing
    */
   public default void setExecutionDelayTime(double delayTime)
   {
      throw new NotImplementedException(getClass().getSimpleName() + " does not implement setExecutionDelayTime");
   }
   
   /**
    * tells the controller if this command supports delayed execution
    * @return
    */
   public default boolean isDelayedExecutionSupported()
   {
      return false;
   }
   
   /**
    * returns the expected execution time of this command. The execution time will be computed when the controller 
    * receives the command using the controllers time plus the execution delay time.
    * This is used when {@code getExecutionDelayTime} is non-zero
    */
   public default double getExecutionTime()
   {
      return 0.0;
   }

   /**
    * sets the execution time for this command. This is called by the controller when the command is received.
    */
   public default void setExecutionTime(double adjustedExecutionTime)
   {
      throw new NotImplementedException(getClass().getSimpleName() + " does not implement setExecutionTime");
   }
}
