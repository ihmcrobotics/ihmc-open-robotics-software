package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.packets.AbstractDesiredAccelerationsMessage;

public abstract class DesiredAccelerationCommand<T extends DesiredAccelerationCommand<T, M>, M extends AbstractDesiredAccelerationsMessage<M>> implements Command<T, M>
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
   public void set(M message)
   {
      desiredJointAccelerations.reset();
      for (int i = 0; i < message.getNumberOfJoints(); i++)
      {
         desiredJointAccelerations.add(message.getDesiredJointAcceleration(i));
      }
   }

   @Override
   public void set(T other)
   {
      desiredJointAccelerations.reset();
      for (int i = 0; i < other.getNumberOfJoints(); i++)
         desiredJointAccelerations.add(other.getDesiredJointAcceleration(i));
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
}
