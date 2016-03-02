package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import us.ihmc.humanoidRobotics.communication.packets.walking.AbortWalkingMessage;

public class ModifiableAbortWalkingMessage implements ControllerMessage<ModifiableAbortWalkingMessage, AbortWalkingMessage>
{
   private boolean abortWalkingRequested = false;

   @Override
   public void clear()
   {
      abortWalkingRequested = false;
   }

   @Override
   public void set(ModifiableAbortWalkingMessage other)
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
}
