package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.AutomaticManipulationAbortMessage;
import us.ihmc.communication.controllerAPI.command.Command;

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
   public void setFromMessage(AutomaticManipulationAbortMessage message)
   {
      enable = message.getEnable();
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
