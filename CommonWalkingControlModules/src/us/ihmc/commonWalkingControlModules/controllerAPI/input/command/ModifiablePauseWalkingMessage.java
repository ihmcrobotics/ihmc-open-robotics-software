package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import us.ihmc.humanoidRobotics.communication.packets.walking.PauseWalkingMessage;

public class ModifiablePauseWalkingMessage implements ControllerMessage<ModifiablePauseWalkingMessage, PauseWalkingMessage>
{
   private boolean isPauseRequested = false;

   public ModifiablePauseWalkingMessage()
   {
   }

   @Override
   public void clear()
   {
      isPauseRequested = false;
   }

   @Override
   public void set(ModifiablePauseWalkingMessage other)
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
