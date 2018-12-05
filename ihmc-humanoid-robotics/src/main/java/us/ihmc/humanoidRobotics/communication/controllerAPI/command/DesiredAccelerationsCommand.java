package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.DesiredAccelerationsMessage;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.communication.controllerAPI.command.QueueableCommand;

public final class DesiredAccelerationsCommand extends QueueableCommand<DesiredAccelerationsCommand, DesiredAccelerationsMessage>
{
   private final TDoubleArrayList desiredJointAccelerations = new TDoubleArrayList(10);

   public DesiredAccelerationsCommand()
   {
   }

   @Override
   public void clear()
   {
      desiredJointAccelerations.reset();
   }

   @Override
   public void setFromMessage(DesiredAccelerationsMessage message)
   {
      desiredJointAccelerations.reset();
      for (int i = 0; i < message.getDesiredJointAccelerations().size(); i++)
      {
         desiredJointAccelerations.add(message.getDesiredJointAccelerations().get(i));
      }
      setQueueableCommandVariables(message.getQueueingProperties());
   }

   @Override
   public void set(DesiredAccelerationsCommand other)
   {
      desiredJointAccelerations.reset();
      for (int i = 0; i < other.getNumberOfJoints(); i++)
         desiredJointAccelerations.add(other.getDesiredJointAcceleration(i));
      setQueueableCommandVariables(other);
   }

   @Override
   public void addTimeOffset(double timeOffset)
   {
   }

   public int getNumberOfJoints()
   {
      return desiredJointAccelerations.size();
   }

   public double getDesiredJointAcceleration(int jointIndex)
   {
      return desiredJointAccelerations.get(jointIndex);
   }

   public TDoubleArrayList getNeckDesiredJointAccelerations()
   {
      return desiredJointAccelerations;
   }

   @Override
   public boolean isCommandValid()
   {
      return !desiredJointAccelerations.isEmpty();
   }

   @Override
   public Class<DesiredAccelerationsMessage> getMessageClass()
   {
      return DesiredAccelerationsMessage.class;
   }
}
