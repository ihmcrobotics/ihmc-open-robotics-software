package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.MimicManeuverCommandMessage;
import us.ihmc.communication.controllerAPI.command.Command;

public class MimicManeuverCommand implements Command<MimicManeuverCommand, MimicManeuverCommandMessage>
{
   private byte requestedAction;
   private boolean execute;

   @Override
   public void set(MimicManeuverCommand other)
   {
      clear();
      requestedAction = other.requestedAction;
      execute = other.execute;
   }

   @Override
   public void setFromMessage(MimicManeuverCommandMessage message)
   {
      clear();
      requestedAction = message.getRequestedAction();
      execute = message.getExecute();
   }

   @Override
   public void clear()
   {
      requestedAction = MimicManeuverCommandMessage.ACTION_NONE;
      execute = false;
   }

   @Override
   public Class<MimicManeuverCommandMessage> getMessageClass()
   {
      return MimicManeuverCommandMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
   }

   @Override
   public long getSequenceId()
   {
      return 0;
   }

   public byte getRequestedAction()
   {
      return requestedAction;
   }

   public boolean getExecute()
   {
      return execute;
   }
}
