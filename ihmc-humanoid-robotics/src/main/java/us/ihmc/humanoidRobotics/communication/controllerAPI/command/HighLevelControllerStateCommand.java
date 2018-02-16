package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.packets.HighLevelStateMessage;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;

public class HighLevelControllerStateCommand implements Command<HighLevelControllerStateCommand, HighLevelStateMessage>
{
   private HighLevelControllerName highLevelControllerName;

   @Override
   public void clear()
   {
      highLevelControllerName = null;
   }

   @Override
   public void set(HighLevelControllerStateCommand other)
   {
      highLevelControllerName = other.getHighLevelControllerName();
   }

   @Override
   public void set(HighLevelStateMessage message)
   {
      highLevelControllerName = HighLevelControllerName.fromByte(message.getHighLevelControllerName());
   }

   public void setHighLevelControllerName(HighLevelControllerName highLevelControllerName)
   {
      this.highLevelControllerName = highLevelControllerName;
   }

   public HighLevelControllerName getHighLevelControllerName()
   {
      return highLevelControllerName;
   }

   @Override
   public Class<HighLevelStateMessage> getMessageClass()
   {
      return HighLevelStateMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return highLevelControllerName != null;
   }
}
