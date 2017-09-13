package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.packets.HighLevelStateMessage;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelController;

public class HighLevelControllerStateCommand implements Command<HighLevelControllerStateCommand, HighLevelStateMessage>
{
   private HighLevelController highLevelController;

   @Override
   public void clear()
   {
      highLevelController = null;
   }

   @Override
   public void set(HighLevelControllerStateCommand other)
   {
      highLevelController = other.getHighLevelController();
   }

   @Override
   public void set(HighLevelStateMessage message)
   {
      highLevelController = message.getHighLevelController();
   }

   public void setHighLevelController(HighLevelController highLevelController)
   {
      this.highLevelController = highLevelController;
   }

   public HighLevelController getHighLevelController()
   {
      return highLevelController;
   }

   @Override
   public Class<HighLevelStateMessage> getMessageClass()
   {
      return HighLevelStateMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return highLevelController != null;
   }
}
