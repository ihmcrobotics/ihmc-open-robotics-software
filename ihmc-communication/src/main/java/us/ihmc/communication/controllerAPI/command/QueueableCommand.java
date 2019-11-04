package us.ihmc.communication.controllerAPI.command;

import controller_msgs.msg.dds.QueueableMessage;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;

/**
 * A QueueableCommand is a {@link Command} that can be queued for execution inside the controller.
 * It implements command IDs that are used to ensure no commands were dropped in the network.
 *
 * @author Georg
 * @param <C> Type of the final implementation of this command (see {@link Command}).
 * @param <M> Type of the network message associated with this command (see {@link Command}).
 */
public abstract class QueueableCommand<C extends QueueableCommand<C, M>, M extends Settable<M>> implements Command<C, M>
{
   /** The ID of this command. Used to make sure only consecutive commands are queued. */
   private long commandId = Packet.VALID_MESSAGE_DEFAULT_ID;
   /** The {@link ExecutionMode} of this command. */
   private ExecutionMode executionMode = ExecutionMode.OVERRIDE;
   /** The ID of the previous command. Used to make sure only consecutive commands are queued. */
   private long previousCommandId = Packet.INVALID_MESSAGE_ID;
   /** the time to delay this command on the controller side before being executed **/
   private double executionDelayTime;
   /** the execution time. This number is set if the execution delay is non zero **/
   public double adjustedExecutionTime;
   /**
    * When receiving a trajectory message that is part of a stream, the controller will extrapolate the
    * trajectory point in the future using a simple first order integration over the given duration.
    * This integration allows to improve continuity of execution for streams. If no new message is
    * received once the integration duration has elapsed, the controller will hold the desired position
    * and reset the desired velocity to 0.
    */
   public double streamIntegrationDuration;

   /**
    * Clears all variables associated with command queuing and sets them to their default values.
    */
   public void clearQueuableCommandVariables()
   {
      commandId = Packet.VALID_MESSAGE_DEFAULT_ID;
      executionMode = ExecutionMode.OVERRIDE;
      previousCommandId = Packet.INVALID_MESSAGE_ID;
      executionDelayTime = 0.0;
      adjustedExecutionTime = 0.0;
      streamIntegrationDuration = 0.0;
   }

   /**
    * Copies the variables associated with command queuing from the given {@link QueueableCommand} into
    * this one.
    */
   public void setQueueableCommandVariables(QueueableCommand<?, ?> other)
   {
      commandId = other.getCommandId();
      executionMode = other.getExecutionMode();
      previousCommandId = other.getPreviousCommandId();
      executionDelayTime = other.getExecutionDelayTime();
      adjustedExecutionTime = other.getExecutionTime();
      streamIntegrationDuration = other.getStreamIntegrationDuration();
   }

   /**
    * Copies the variables associated with command queuing from the given {@link QueueableMessage} into
    * this one.
    */
   public void setQueueableCommandVariables(QueueableMessage messageQueueingProperties)
   {
      if (messageQueueingProperties == null)
         return;
      commandId = messageQueueingProperties.getMessageId();
      executionMode = ExecutionMode.fromByte(messageQueueingProperties.getExecutionMode());
      previousCommandId = messageQueueingProperties.getPreviousMessageId();
      executionDelayTime = messageQueueingProperties.getExecutionDelayTime();
      streamIntegrationDuration = messageQueueingProperties.getStreamIntegrationDuration();
   }

   /**
    * Sets the {@link #commandId} of this command. The ID is used for making sure that only consecutive
    * commands are queued.
    */
   public void setCommandId(long commandId)
   {
      this.commandId = commandId;
   }

   /**
    * Sets the {@link #previousCommandId} of this command. The ID is used for making sure that only
    * consecutive commands are queued.
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
    * Gets the {@link #commandId} of this command. The ID is used for making sure that only consecutive
    * commands are queued.
    */
   public long getCommandId()
   {
      return commandId;
   }

   /**
    * Gets the {@link #previousCommandId} of this command. The ID is used for making sure that only
    * consecutive commands are queued.
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
    * returns the amount of time this command is delayed on the controller side before executing
    * 
    * @return the time to delay this command in seconds
    */
   @Override
   public double getExecutionDelayTime()
   {
      return executionDelayTime;
   }

   /**
    * sets the amount of time this command is delayed on the controller side before executing
    * 
    * @param delayTime the time in seconds to delay after receiving the command before executing
    */
   @Override
   public void setExecutionDelayTime(double delayTime)
   {
      this.executionDelayTime = delayTime;
   }

   /**
    * Checks if a valid execution mode was set for the command.
    */
   public boolean executionModeValid()
   {
      return executionMode != null;
   }

   /**
    * Used to offset the trajectory in case it is queued.
    */
   public abstract void addTimeOffset(double timeOffset);

   /**
    * returns the expected execution time of this command. The execution time will be computed when the
    * controller receives the command using the controllers time plus the execution delay time. This is
    * used when {@code getExecutionDelayTime} is non-zero
    */
   @Override
   public double getExecutionTime()
   {
      return adjustedExecutionTime;
   }

   /**
    * sets the execution time for this command. This is called by the controller when the command is
    * received.
    */
   @Override
   public void setExecutionTime(double adjustedExecutionTime)
   {
      this.adjustedExecutionTime = adjustedExecutionTime;
   }

   /**
    * tells the controller if this command supports delayed execution (Spoiler alert: It does)
    */
   @Override
   public boolean isDelayedExecutionSupported()
   {
      return true;
   }

   /**
    * When receiving a trajectory message that is part of a stream, the controller will extrapolate the
    * trajectory point in the future using a simple first order integration over the given duration.
    * This integration allows to improve continuity of execution for streams. If no new message is
    * received once the integration duration has elapsed, the controller will hold the desired position
    * and reset the desired velocity to 0.
    */
   public double getStreamIntegrationDuration()
   {
      return streamIntegrationDuration;
   }

   public boolean epsilonEquals(C other, double epsilon)
   {
      if (commandId != other.getCommandId())
      {
         return false;
      }
      if (executionMode != other.getExecutionMode())
      {
         return false;
      }
      if (previousCommandId != other.getPreviousCommandId())
      {
         return false;
      }
      if (executionDelayTime != other.getExecutionDelayTime())
      {
         return false;
      }
      if (adjustedExecutionTime != other.getExecutionTime())
      {
         return false;
      }
      if (streamIntegrationDuration != other.getStreamIntegrationDuration())
      {
         return false;
      }
      return true;
   }
}
