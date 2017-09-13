package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.packets.HighLevelStateMessage;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerState;

public class HighLevelControllerStateCommand implements Command<HighLevelControllerStateCommand, HighLevelStateMessage>
{
   private HighLevelControllerState highLevelControllerState;

   @Override
   public void clear()
   {
      highLevelControllerState = null;
   }

   @Override
   public void set(HighLevelControllerStateCommand other)
   {
      highLevelControllerState = other.getHighLevelControllerState();
   }

   @Override
   public void set(HighLevelStateMessage message)
   {
      highLevelControllerState = message.getHighLevelControllerState();
   }

   public void setHighLevelControllerState(HighLevelControllerState highLevelControllerState)
   {
      this.highLevelControllerState = highLevelControllerState;
   }

   public HighLevelControllerState getHighLevelControllerState()
   {
      return highLevelControllerState;
   }

   @Override
   public Class<HighLevelStateMessage> getMessageClass()
   {
      return HighLevelStateMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return highLevelControllerState != null;
   }
}
