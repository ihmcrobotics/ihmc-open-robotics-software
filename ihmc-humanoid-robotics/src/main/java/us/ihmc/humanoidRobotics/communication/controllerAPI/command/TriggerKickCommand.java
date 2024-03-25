package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.TriggerKickMessage;
import us.ihmc.communication.controllerAPI.command.Command;

public class TriggerKickCommand implements Command<TriggerKickCommand, TriggerKickMessage>
{
   private long sequenceId;
   private boolean triggerKickRequested = false;

   @Override
   public void clear()
   {
      sequenceId = 0;
      triggerKickRequested = false;
   }

   @Override
   public void set(TriggerKickCommand other)
   {
      sequenceId = other.sequenceId;
      triggerKickRequested = other.isTriggerKickRequested();
   }

   @Override
   public void setFromMessage(TriggerKickMessage message)
   {
      sequenceId = message.getSequenceId();
      triggerKickRequested = true;
   }

   public boolean isTriggerKickRequested()
   {
      return triggerKickRequested;
   }

   @Override
   public Class<TriggerKickMessage> getMessageClass()
   {
      return TriggerKickMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
