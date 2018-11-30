package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.PauseWalkingMessage;
import us.ihmc.communication.controllerAPI.command.Command;

public class PauseWalkingCommand implements Command<PauseWalkingCommand, PauseWalkingMessage>
{
   private boolean isPauseRequested = false;

   public PauseWalkingCommand()
   {
   }

   @Override
   public void clear()
   {
      isPauseRequested = false;
   }

   @Override
   public void set(PauseWalkingCommand other)
   {
      isPauseRequested = other.isPauseRequested;
   }

   @Override
   public void setFromMessage(PauseWalkingMessage message)
   {
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
}
