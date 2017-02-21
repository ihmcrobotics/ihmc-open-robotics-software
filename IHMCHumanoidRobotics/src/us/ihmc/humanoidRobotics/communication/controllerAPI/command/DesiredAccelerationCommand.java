package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.Packet;

public abstract class DesiredAccelerationCommand<T extends DesiredAccelerationCommand<T, M>, M extends Packet<M>> implements Command<T, M>
{
   private final TDoubleArrayList desiredJointAccelerations = new TDoubleArrayList(10);
   private double weight;

   public DesiredAccelerationCommand()
   {
   }

   @Override
   public void clear()
   {
      desiredJointAccelerations.reset();
      weight = 0;
   }

   @Override
   public void set(M message)
   {
//      weight = message.getWeight();
//      desiredJointAccelerations.reset();
//      for (int i = 0; i < message.getNumberOfJoints(); i++)
//      {
//         desiredJointAccelerations.add(message.getNeckDesiredJointAcceleration(i));
//      }
   }

   @Override
   public void set(T other)
   {
      weight = other.getWeight();
      desiredJointAccelerations.reset();
      for (int i = 0; i < other.getNumberOfJoints(); i++)
         desiredJointAccelerations.add(other.getDesiredJointAcceleration(i));
   }
   
   public double getWeight()
   {
      return weight;
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
