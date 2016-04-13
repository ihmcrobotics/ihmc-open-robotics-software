package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.packets.walking.AutomaticManipulationAbortMessage;

public class AutomaticManipulationAbortCommand implements Command<AutomaticManipulationAbortCommand, AutomaticManipulationAbortMessage>
{
   private boolean enable;

   public AutomaticManipulationAbortCommand()
   {
   }

   @Override
   public void clear()
   {
   }

   @Override
   public void set(AutomaticManipulationAbortCommand other)
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
