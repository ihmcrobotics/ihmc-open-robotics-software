package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.packets.NewHighLevelControllerStateMessage;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates;

public class NewHighLevelStateCommand implements Command<NewHighLevelStateCommand, NewHighLevelControllerStateMessage>
{
   private NewHighLevelControllerStates highLevelState;

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

   public void setHighLevelState(NewHighLevelControllerStates highLevelState)
   {
      this.highLevelState = highLevelState;
   }

   public NewHighLevelControllerStates getHighLevelState()
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
