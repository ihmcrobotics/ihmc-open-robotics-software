package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.communication.controllerAPI.command.QueueableCommand;
import us.ihmc.humanoidRobotics.communication.packets.AbstractDesiredAccelerationsMessage;

public final class DesiredAccelerationCommand extends QueueableCommand<DesiredAccelerationCommand, AbstractDesiredAccelerationsMessage>
{
   private final TDoubleArrayList desiredJointAccelerations = new TDoubleArrayList(10);

   public DesiredAccelerationCommand()
   {
   }

   @Override
   public void clear()
   {
      desiredJointAccelerations.reset();
   }

   @Override
   public void set(AbstractDesiredAccelerationsMessage message)
   {
      desiredJointAccelerations.reset();
      for (int i = 0; i < message.getNumberOfJoints(); i++)
      {
         desiredJointAccelerations.add(message.getDesiredJointAcceleration(i));
      }
      setQueueableCommandVariables(message.getUniqueId(), message.getQueueingProperties());
   }

   @Override
   public void set(DesiredAccelerationCommand other)
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
   public Class<AbstractDesiredAccelerationsMessage> getMessageClass()
   {
      return AbstractDesiredAccelerationsMessage.class;
   }
}
