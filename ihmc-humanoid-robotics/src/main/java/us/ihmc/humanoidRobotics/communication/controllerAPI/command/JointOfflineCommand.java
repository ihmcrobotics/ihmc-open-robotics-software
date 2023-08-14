package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.JointOfflineMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;

public class JointOfflineCommand implements Command<JointOfflineCommand, JointOfflineMessage>
{
   private long sequenceId;

   /* Hash-code of the joint that is offline/dead */
   private int jointOfflineHashCode;

   public JointOfflineCommand()
   {
      clear();
   }

   public int getJointOfflineHashCode()
   {
      return jointOfflineHashCode;
   }

   public void setJointOfflineHashCode(int jointOfflineHashCode)
   {
      this.jointOfflineHashCode = jointOfflineHashCode;
   }

   @Override
   public void set(JointOfflineCommand other)
   {
      sequenceId = other.sequenceId;
      jointOfflineHashCode = other.jointOfflineHashCode;
   }

   @Override
   public void setFromMessage(JointOfflineMessage message)
   {
      sequenceId = message.getSequenceId();
      jointOfflineHashCode = message.getJointOfflineHashCode();
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      jointOfflineHashCode = -1;
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
   }

   @Override
   public Class<JointOfflineMessage> getMessageClass()
   {
      return JointOfflineMessage.class;
   }

   public void setSequenceId(long sequenceId)
   {
      this.sequenceId = sequenceId;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }

}
