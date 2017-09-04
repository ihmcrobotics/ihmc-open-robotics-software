package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.packets.NewHighLevelControllerStateMessage;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates;

public class NewHighLevelControllerStateCommand implements Command<NewHighLevelControllerStateCommand, NewHighLevelControllerStateMessage>
{
   private NewHighLevelControllerStates highLevelControllerState;

   @Override
   public void clear()
   {
      highLevelControllerState = null;
   }

   @Override
   public void set(NewHighLevelControllerStateCommand other)
   {
      highLevelControllerState = other.getHighLevelControllerState();
   }

   @Override
   public void set(NewHighLevelControllerStateMessage message)
   {
      highLevelControllerState = message.getHighLevelState();
   }

   public void setHighLevelControllerState(NewHighLevelControllerStates highLevelControllerState)
   {
      this.highLevelControllerState = highLevelControllerState;
   }

   public NewHighLevelControllerStates getHighLevelControllerState()
   {
      return highLevelControllerState;
   }

   @Override
   public Class<NewHighLevelControllerStateMessage> getMessageClass()
   {
      return NewHighLevelControllerStateMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return highLevelControllerState != null;
   }
}
