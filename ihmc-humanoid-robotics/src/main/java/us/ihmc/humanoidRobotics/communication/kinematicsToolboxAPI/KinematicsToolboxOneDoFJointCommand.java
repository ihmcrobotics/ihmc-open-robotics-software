package us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI;

import java.util.Objects;

import toolbox_msgs.msg.dds.KinematicsToolboxOneDoFJointMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.JointHashCodeResolver;

public class KinematicsToolboxOneDoFJointCommand implements Command<KinematicsToolboxOneDoFJointCommand, KinematicsToolboxOneDoFJointMessage>
{
   private long sequenceId;
   private OneDoFJointBasics joint;
   private double desiredPosition;
   private double weight;

   @Override
   public void clear()
   {
      sequenceId = 0;
      joint = null;
      desiredPosition = 0.0;
      weight = Double.NaN;
   }

   @Override
   public void set(KinematicsToolboxOneDoFJointCommand other)
   {
      sequenceId = other.sequenceId;
      joint = other.joint;
      desiredPosition = other.desiredPosition;
      weight = other.weight;
   }

   @Override
   public void setFromMessage(KinematicsToolboxOneDoFJointMessage message)
   {
      set(message, null);
   }

   public void set(KinematicsToolboxOneDoFJointMessage message, JointHashCodeResolver jointHashCodeResolver)
   {
      Objects.requireNonNull(jointHashCodeResolver);

      sequenceId = message.getSequenceId();
      joint = jointHashCodeResolver.castAndGetJoint(message.getJointHashCode());
      desiredPosition = message.getDesiredPosition();
      weight = message.getWeight();
   }

   public void setJoint(OneDoFJointBasics joint)
   {
      this.joint = joint;
   }

   public void setDesiredPosition(double desiredPosition)
   {
      this.desiredPosition = desiredPosition;
   }

   public void setWeight(double weight)
   {
      this.weight = weight;
   }

   public OneDoFJointBasics getJoint()
   {
      return joint;
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
