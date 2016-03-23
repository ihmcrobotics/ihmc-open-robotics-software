package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import us.ihmc.humanoidRobotics.communication.packets.walking.AutomaticManipulationAbortMessage;

public class AutomaticManipulationAbortControllerCommand implements ControllerCommand<AutomaticManipulationAbortControllerCommand, AutomaticManipulationAbortMessage>
{
   private boolean enable;

   public AutomaticManipulationAbortControllerCommand()
   {
   }

   @Override
   public void clear()
   {
   }

   @Override
   public void set(AutomaticManipulationAbortControllerCommand other)
   {
      enable = other.enable;
   }

   @Override
   public void set(AutomaticManipulationAbortMessage message)
   {
      enable = message.enable;
   }

   public boolean isEnable()
   {
      return enable;
   }

   public void setEnable(boolean enable)
   {
      this.enable = enable;
   }

   @Override
   public Class<AutomaticManipulationAbortMessage> getMessageClass()
   {
      return AutomaticManipulationAbortMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
   }
}
