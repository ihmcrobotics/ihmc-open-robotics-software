package us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI;

import controller_msgs.msg.dds.KinematicsToolboxOneDoFJointMessage;
import us.ihmc.communication.controllerAPI.command.Command;

public class KinematicsToolboxOneDoFJointCommand implements Command<KinematicsToolboxOneDoFJointCommand, KinematicsToolboxOneDoFJointMessage>
{
   private long sequenceId;
   /** This is the unique hash code of the joint to be solved for. */
   private int jointHashCode;

   private double desiredPosition;
   private double weight;

   @Override
   public void clear()
   {
      sequenceId = 0;
      jointHashCode = 0;
      desiredPosition = 0.0;
      weight = Double.NaN;
   }

   @Override
   public void set(KinematicsToolboxOneDoFJointCommand other)
   {
      sequenceId = other.sequenceId;
      jointHashCode = other.jointHashCode;
      desiredPosition = other.desiredPosition;
      weight = other.weight;
   }

   @Override
   public void setFromMessage(KinematicsToolboxOneDoFJointMessage message)
   {
      sequenceId = message.getSequenceId();
      jointHashCode = message.getJointHashCode();
      desiredPosition = message.getDesiredPosition();
      weight = message.getWeight();
   }

   public int getJointHashCode()
   {
      return jointHashCode;
   }

   public double getDesiredPosition()
   {
      return desiredPosition;
   }

   public double getWeight()
   {
      return weight;
   }

   @Override
   public Class<KinematicsToolboxOneDoFJointMessage> getMessageClass()
   {
      return KinematicsToolboxOneDoFJointMessage.class;
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
