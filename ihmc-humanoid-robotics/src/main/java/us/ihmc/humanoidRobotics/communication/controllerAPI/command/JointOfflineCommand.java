package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.JointOfflineMessage;
import controller_msgs.msg.dds.LoadBearingMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;

public final class JointOfflineCommand implements Command<JointOfflineCommand, JointOfflineMessage>
{
   private long sequenceId;

   /** the joint which should be considered unusable/dead */
   private OneDoFJointBasics jointToGoOffline;

   public JointOfflineCommand()
   {
      //TODO This constructor needs to exist, is it ok to do nothing here?
   }

   public JointOfflineCommand(OneDoFJointBasics jointToGoOffline)
   {
      this.jointToGoOffline = jointToGoOffline;
   }

   public OneDoFJointBasics getJointToGoOffline()
   {
      return jointToGoOffline;
   }

   @Override
   public void set(JointOfflineCommand other)
   {
      //TODO set all fields to 0 and null
      sequenceId = other.sequenceId;
      // set jointToGoOffline to null or do nothing
   }

   @Override
   public void setFromMessage(JointOfflineMessage message)
   {
      //TODO if anything is done with the message, update this
      //      sequenceId = message.getSequenceId();
      //      jointToGoOffline = message.getJointToGoOffline();
   }

   @Override
   public void clear()
   {
      //TODO copied from foot load bearing message
      sequenceId = 0;
      //      jointToGoOffline = null; //dont do this
   }

   @Override
   public boolean isCommandValid()
   {
      if (jointToGoOffline == null)
         return false;
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
