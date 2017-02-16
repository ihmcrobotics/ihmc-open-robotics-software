package us.ihmc.communication.controllerAPI.command;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.QueueableMessage;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;

/**
 * A QueueableCommand is a {@link #Command} that can be queued for execution inside the controller. It implements command
 * IDs that are used to ensure no commands were dropped in the network.
 *
 * @author Georg
 *
 * @param <C> Type of the final implementation of this command (see {@link #Command}).
 * @param <M> Type of the network message associated with this command (see {@link #Command}).
 */
public abstract class QueueableCommand<C extends QueueableCommand<C, M>, M extends QueueableMessage<M>> implements Command<C, M>
{
   /** The ID of this command. Used to make sure only consecutive commands are queued. */
   private long commandId = Packet.VALID_MESSAGE_DEFAULT_ID;
   /** The ID of the previous command. Used to make sure only consecutive commands are queued. */
   private long previousCommandId = Packet.INVALID_MESSAGE_ID;
   /** The {@link #ExecutionMode} of this command. */
   private ExecutionMode executionMode = ExecutionMode.OVERRIDE;

   /**
    * Clears all variables associated with command queuing and sets them to their default values.
    */
   public void clearQueuableCommandVariables()
   {
      commandId = Packet.VALID_MESSAGE_DEFAULT_ID;
      executionMode = ExecutionMode.OVERRIDE;
      previousCommandId = Packet.INVALID_MESSAGE_ID;
   }

   /**
    * Copies the variables associated with command queuing from the given {@link #QueueableCommand} into this one.
    */
   protected void setQueueqableCommandVariables(QueueableCommand<?, ?> other)
   {
      commandId = other.getCommandId();
      executionMode = other.getExecutionMode();
      previousCommandId = other.getPreviousCommandId();
   }

   /**
    * Copies the variables associated with command queuing from the given {@link #QueueableMessage} into this one.
    */
   protected void setQueueqableCommandVariables(QueueableMessage<?> message)
   {
      commandId = message.getUniqueId();
      executionMode = message.getExecutionMode();
      previousCommandId = message.getPreviousMessageId();
   }

   /**
    * Sets the {@link #commandId} of this command. The ID is used for making sure that only consecutive commands are queued.
    */
   public void setCommandId(long commandId)
   {
      this.commandId = commandId;
   }

   /**
    * Sets the {@link #previousCommandId} of this command. The ID is used for making sure that only consecutive commands are queued.
    */
   public void setPreviousCommandId(long previousCommandId)
   {
      this.previousCommandId = previousCommandId;
   }

   /**
    * Sets the {@link #executionMode} of this command.
    */
   public void setExecutionMode(ExecutionMode executionMode)
   {
      this.executionMode = executionMode;
   }

   /**
    * Gets the {@link #commandId} of this command. The ID is used for making sure that only consecutive commands are queued.
    */
   public long getCommandId()
   {
      return commandId;
   }

   /**
    * Gets the {@link #previousCommandId} of this command. The ID is used for making sure that only consecutive commands are queued.
    */
   public long getPreviousCommandId()
   {
      return previousCommandId;
   }

   /**
    * Gets the {@link #executionMode} of this command.
    */
   public ExecutionMode getExecutionMode()
   {
      return executionMode;
   }

   /**
    * Checks if a valid execution mode was set for the command.
    */
   public boolean executionModeValid()
   {
      return executionMode != null;
   }
}
