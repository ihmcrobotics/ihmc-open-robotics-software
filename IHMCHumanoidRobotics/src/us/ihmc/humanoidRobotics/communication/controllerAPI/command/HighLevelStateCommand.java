package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.packets.HighLevelStateMessage;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelState;

public class HighLevelStateCommand implements Command<HighLevelStateCommand, HighLevelStateMessage>
{
   private HighLevelState highLevelState;

   @Override
   public void clear()
   {
      highLevelState = null;
   }

   @Override
   public void set(HighLevelStateCommand other)
   {
      highLevelState = other.getHighLevelState();
   }

   @Override
   public void set(HighLevelStateMessage message)
   {
      highLevelState = message.getHighLevelState();
   }

   public void setHighLevelState(HighLevelState highLevelState)
   {
      this.highLevelState = highLevelState;
   }

   public HighLevelState getHighLevelState()
   {
      return highLevelState;
   }

   @Override
   public Class<HighLevelStateMessage> getMessageClass()
   {
      return HighLevelStateMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return highLevelState != null;
   }
}
