package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.AbortWalkingMessage;
import us.ihmc.communication.controllerAPI.command.Command;

public class AbortWalkingCommand implements Command<AbortWalkingCommand, AbortWalkingMessage>
{
   private long sequenceId;
   private boolean abortWalkingRequested = false;

   @Override
   public void clear()
   {
      sequenceId = 0;
      abortWalkingRequested = false;
   }

   @Override
   public void set(AbortWalkingCommand other)
   {
      sequenceId = other.sequenceId;
      abortWalkingRequested = other.isAbortWalkingRequested();
   }

   @Override
   public void setFromMessage(AbortWalkingMessage message)
   {
      sequenceId = message.getSequenceId();
      abortWalkingRequested = true;
   }

   public boolean isAbortWalkingRequested()
   {
      return abortWalkingRequested;
   }

   @Override
   public Class<AbortWalkingMessage> getMessageClass()
   {
      return AbortWalkingMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
