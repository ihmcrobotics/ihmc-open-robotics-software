package us.ihmc.communication.packets;

import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;

/**
 * A QueueableMessage is a {@link #Packet} that can be queued for execution inside the controller. It implements command
 * IDs that are used to ensure no commands were dropped in the network.
 *
 * @author Georg
 *
 * @param <T> Type of the final implementation of this message.
 */
public abstract class QueueableMessage<T extends QueueableMessage<T>> extends TrackablePacket<T>
{
   @RosExportedField(documentation = "When OVERRIDE is chosen:"
         + "\n - The time of the first trajectory point can be zero, in which case the controller will start directly at the first trajectory point."
         + " Otherwise the controller will prepend a first trajectory point at the current desired position." + "\n When QUEUE is chosen:"
         + "\n - The message must carry the ID of the message it should be queued to."
         + "\n - The very first message of a list of queued messages has to be an OVERRIDE message."
         + "\n - The trajectory point times are relative to the the last trajectory point time of the previous message."
         + "\n - The controller will queue the joint trajectory messages as a per joint basis." + " The first trajectory point has to be greater than zero.")
   public ExecutionMode executionMode = ExecutionMode.OVERRIDE;
   @RosExportedField(documentation = "Only needed when using QUEUE mode, it refers to the message Id to which this message should be queued to."
         + " It is used by the controller to ensure that no message has been lost on the way."
         + " If a message appears to be missing (previousMessageId different from the last message ID received by the controller), the motion is aborted."
         + " If previousMessageId == 0, the controller will not check for the ID of the last received message.")
   public long previousMessageId = INVALID_MESSAGE_ID;

   /**
    * Empty constructor for serialization.
    */
   public QueueableMessage()
   {
   }

   /**
    * Random constructor for message tests.
    * @param random
    */
   public QueueableMessage(Random random)
   {
      executionMode = RandomNumbers.nextEnum(random, ExecutionMode.class);
   }

   /**
    * Set how the controller should consume this message:
    * <li> {@link ExecutionMode#OVERRIDE}: this message will override any previous message, including canceling any active execution of a message.
    * <li> {@link ExecutionMode#QUEUE}: this message is queued and will be executed once all the previous messages are done.
    * @param executionMode
    * @param previousMessageId when queuing, one needs to provide the ID of the message this message should be queued to.
    */
   public void setExecutionMode(ExecutionMode executionMode, long previousMessageId)
   {
      this.executionMode = executionMode;
      this.previousMessageId = previousMessageId;
   }

   /**
    * Returns the execution mode of the packet. This will tell the controller whether the message should be queued or executed immediately.
    * @return {@link #executionMode}
    */
   public ExecutionMode getExecutionMode()
   {
      return executionMode;
   }

   /**
    * Returns the previous message ID. If the message is queued this is used to verify that no packets were dropped.
    * @return {@link #previousMessageId}
    */
   public long getPreviousMessageId()
   {
      return previousMessageId;
   }

   /** {@inheritDoc} */
   @Override
   public boolean epsilonEquals(T other, double epsilon)
   {
      if (executionMode != other.getExecutionMode())
         return false;
      if (executionMode == ExecutionMode.OVERRIDE && previousMessageId != other.getPreviousMessageId())
         return false;
      return true;
   }
}
