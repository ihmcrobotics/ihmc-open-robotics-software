package us.ihmc.quadrupedRobotics.communication.commands;

import controller_msgs.msg.dds.QuadrupedRequestedControllerStateMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerRequestedEvent;

public class QuadrupedRequestedControllerStateCommand implements Command<QuadrupedRequestedControllerStateCommand, QuadrupedRequestedControllerStateMessage>
{
   private QuadrupedForceControllerRequestedEvent requestedControllerState;

   @Override
   public void clear()
   {
      requestedControllerState = null;
   }

   @Override
   public void set(QuadrupedRequestedControllerStateCommand other)
   {
      requestedControllerState = other.getRequestedControllerState();
   }

   @Override
   public void set(QuadrupedRequestedControllerStateMessage message)
   {
      requestedControllerState = QuadrupedForceControllerRequestedEvent.fromByte(message.getQuadrupedControllerName());
   }

   public void setRequestedControllerState(QuadrupedForceControllerRequestedEvent requestedControllerState)
   {
      this.requestedControllerState = requestedControllerState;
   }

   public QuadrupedForceControllerRequestedEvent getRequestedControllerState()
   {
      return requestedControllerState;
   }

   @Override
   public Class<QuadrupedRequestedControllerStateMessage> getMessageClass()
   {
      return QuadrupedRequestedControllerStateMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return requestedControllerState != null;
   }
}
