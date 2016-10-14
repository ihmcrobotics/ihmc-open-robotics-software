package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.packets.walking.PauseWalkingMessage;

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
   public void set(PauseWalkingMessage message)
   {
      isPauseRequested = message.pause;
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
