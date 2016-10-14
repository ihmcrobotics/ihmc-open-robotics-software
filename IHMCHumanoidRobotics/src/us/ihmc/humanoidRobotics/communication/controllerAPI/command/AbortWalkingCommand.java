package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.packets.walking.AbortWalkingMessage;

public class AbortWalkingCommand implements Command<AbortWalkingCommand, AbortWalkingMessage>
{
   private boolean abortWalkingRequested = false;

   @Override
   public void clear()
   {
      abortWalkingRequested = false;
   }

   @Override
   public void set(AbortWalkingCommand other)
   {
      abortWalkingRequested = other.isAbortWalkingRequested();
   }

   @Override
   public void set(AbortWalkingMessage message)
   {
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
}
