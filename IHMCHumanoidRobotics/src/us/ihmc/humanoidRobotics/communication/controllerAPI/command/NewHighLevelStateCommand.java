package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.packets.NewHighLevelControllerStateMessage;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelStates;

public class NewHighLevelStateCommand implements Command<NewHighLevelStateCommand, NewHighLevelControllerStateMessage>
{
   private NewHighLevelStates highLevelState;

   @Override
   public void clear()
   {
      highLevelState = null;
   }

   @Override
   public void set(NewHighLevelStateCommand other)
   {
      highLevelState = other.getHighLevelState();
   }

   @Override
   public void set(NewHighLevelControllerStateMessage message)
   {
      highLevelState = message.getHighLevelState();
   }

   public void setHighLevelState(NewHighLevelStates highLevelState)
   {
      this.highLevelState = highLevelState;
   }

   public NewHighLevelStates getHighLevelState()
   {
      return highLevelState;
   }

   @Override
   public Class<NewHighLevelControllerStateMessage> getMessageClass()
   {
      return NewHighLevelControllerStateMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return highLevelState != null;
   }
}
