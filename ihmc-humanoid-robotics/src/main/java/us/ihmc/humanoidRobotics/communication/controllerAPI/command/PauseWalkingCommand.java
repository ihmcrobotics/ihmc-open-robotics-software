package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.PauseWalkingMessage;
import us.ihmc.communication.controllerAPI.command.Command;

public class PauseWalkingCommand implements Command<PauseWalkingCommand, PauseWalkingMessage>
{
   private long sequenceId;
   private boolean isPauseRequested = false;

   public PauseWalkingCommand()
   {
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      isPauseRequested = false;
   }

   @Override
   public void set(PauseWalkingCommand other)
   {
      sequenceId = other.sequenceId;
      isPauseRequested = other.isPauseRequested;
   }

   @Override
   public void setFromMessage(PauseWalkingMessage message)
   {
      sequenceId = message.getSequenceId();
      isPauseRequested = message.getPause();
   }

   public boolean isPauseRequested()
   {
      return isPauseRequested;
   }

   public void setPauseRequested(boolean isPauseRequested)
   {
      this.isPauseRequested = isPauseRequested;
   }

   @Override
   public Class<PauseWalkingMessage> getMessageClass()
   {
      return PauseWalkingMessage.class;
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
