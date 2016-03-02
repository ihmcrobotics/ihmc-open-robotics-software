package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import us.ihmc.humanoidRobotics.communication.packets.HighLevelStateMessage;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelState;

public class ModifiableHighLevelStateMessage implements ControllerMessage<ModifiableHighLevelStateMessage, HighLevelStateMessage>
{
   private HighLevelState highLevelState;

   @Override
   public void clear()
   {
      highLevelState = null;
   }

   @Override
   public void set(ModifiableHighLevelStateMessage other)
   {
      highLevelState = other.getHighLevelState();
   }

   @Override
   public void set(HighLevelStateMessage message)
   {
      highLevelState = message.getHighLevelState();
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
}
