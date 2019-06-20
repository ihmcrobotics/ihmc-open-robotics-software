package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.HighLevelStateMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;

public class HighLevelControllerStateCommand implements Command<HighLevelControllerStateCommand, HighLevelStateMessage>
{
   private long sequenceId;
   private HighLevelControllerName highLevelControllerName;

   @Override
   public void clear()
   {
      sequenceId = 0;
      highLevelControllerName = null;
   }

   @Override
   public void set(HighLevelControllerStateCommand other)
   {
      sequenceId = other.sequenceId;
      highLevelControllerName = other.getHighLevelControllerName();
   }

   @Override
   public void setFromMessage(HighLevelStateMessage message)
   {
      sequenceId = message.getSequenceId();
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

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
