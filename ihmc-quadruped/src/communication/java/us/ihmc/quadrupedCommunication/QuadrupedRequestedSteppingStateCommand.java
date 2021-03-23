package us.ihmc.quadrupedCommunication;

import controller_msgs.msg.dds.QuadrupedRequestedSteppingStateMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.quadrupedBasics.QuadrupedSteppingRequestedEvent;

public class QuadrupedRequestedSteppingStateCommand implements Command<QuadrupedRequestedSteppingStateCommand, QuadrupedRequestedSteppingStateMessage>
{
   private long sequenceId;
   private QuadrupedSteppingRequestedEvent requestedSteppingState;

   @Override
   public void clear()
   {
      sequenceId = 0;
      requestedSteppingState = null;
   }

   @Override
   public void set(QuadrupedRequestedSteppingStateCommand other)
   {
      sequenceId = other.sequenceId;
      requestedSteppingState = other.getRequestedSteppingState();
   }

   @Override
   public void setFromMessage(QuadrupedRequestedSteppingStateMessage message)
   {
      sequenceId = message.getSequenceId();
      requestedSteppingState = QuadrupedSteppingRequestedEvent.fromByte(message.getQuadrupedSteppingRequestedEvent());
   }

   public void setRequestedSteppingState(QuadrupedSteppingRequestedEvent requestedSteppingState)
   {
      this.requestedSteppingState = requestedSteppingState;
   }

   public QuadrupedSteppingRequestedEvent getRequestedSteppingState()
   {
      return requestedSteppingState;
   }

   @Override
   public Class<QuadrupedRequestedSteppingStateMessage> getMessageClass()
   {
      return QuadrupedRequestedSteppingStateMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return requestedSteppingState != null;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
