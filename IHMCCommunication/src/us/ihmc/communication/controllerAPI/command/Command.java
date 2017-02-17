package us.ihmc.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.packets.Packet;

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
 * @param <C> Type of the final implementation of this Command. It is used for the copy method {@link #set(Command)}.
 * @param <M> Type of the corresponding {@link Packet} that this Command can copy.
 */
public interface Command<C extends Command<C, M>, M extends Packet<M>>
{
   /**
    * Clear all the data held in this Command.
    * Used when the data has been processed by the controller for instance.
    * This HAS to be garbage free.
    */
   public abstract void clear();

   /**
    * Copy the data from another Command.
    * Calling this method should override the data held in this.
    * @param other Command to be copied.
    */
   public abstract void set(C other);

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
}
