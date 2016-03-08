package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import us.ihmc.humanoidRobotics.communication.packets.walking.PauseWalkingMessage;

public class PauseWalkingControllerCommand implements ControllerCommand<PauseWalkingControllerCommand, PauseWalkingMessage>
{
   private boolean isPauseRequested = false;

   public PauseWalkingControllerCommand()
   {
   }

   @Override
   public void clear()
   {
      isPauseRequested = false;
   }

   @Override
   public void set(PauseWalkingControllerCommand other)
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
}
