package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.RLPolicyCommand;
import us.ihmc.communication.controllerAPI.command.Command;

public class RLPolicyCommandCommand implements Command<RLPolicyCommandCommand, RLPolicyCommand>
{
   private byte desiredModel;
   private boolean executeDesiredModel;

   @Override
   public void set(RLPolicyCommandCommand other)
   {
      clear();
      desiredModel = other.desiredModel;
      executeDesiredModel = other.executeDesiredModel;
   }

   @Override
   public void setFromMessage(RLPolicyCommand message)
   {
      clear();
      desiredModel = message.getDesiredModel();
      executeDesiredModel = message.getExecuteDesiredModel();
   }

   @Override
   public void clear()
   {
      desiredModel = 0;
      executeDesiredModel = false;
   }

   @Override
   public Class<RLPolicyCommand> getMessageClass()
   {
      return RLPolicyCommand.class;
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

   public byte getDesiredModel()
   {
      return desiredModel;
   }

   public boolean getExecuteDesiredModel()
   {
      return executeDesiredModel;
   }
}
