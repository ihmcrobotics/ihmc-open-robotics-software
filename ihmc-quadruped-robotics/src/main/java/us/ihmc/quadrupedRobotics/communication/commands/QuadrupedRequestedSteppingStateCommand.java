package us.ihmc.quadrupedRobotics.communication.commands;

import controller_msgs.msg.dds.QuadrupedRequestedSteppingStateMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedSteppingRequestedEvent;

public class QuadrupedRequestedSteppingStateCommand implements Command<QuadrupedRequestedSteppingStateCommand, QuadrupedRequestedSteppingStateMessage>
{
   private QuadrupedSteppingRequestedEvent requestedSteppingState;

   @Override
   public void clear()
   {
      requestedSteppingState = null;
   }

   @Override
   public void set(QuadrupedRequestedSteppingStateCommand other)
   {
      requestedSteppingState = other.getRequestedSteppingState();
   }

   @Override
   public void set(QuadrupedRequestedSteppingStateMessage message)
   {
      requestedSteppingState = QuadrupedSteppingRequestedEvent.fromByte(message.getQuadrupedSteppingState());
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
}
