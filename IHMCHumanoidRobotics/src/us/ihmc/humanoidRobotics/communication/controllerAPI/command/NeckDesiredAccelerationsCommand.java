package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.packets.walking.NeckDesiredAccelerationsMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.NeckDesiredAccelerationsMessage.NeckControlMode;

public class NeckDesiredAccelerationsCommand
      implements Command<NeckDesiredAccelerationsCommand, NeckDesiredAccelerationsMessage>
{
   private NeckControlMode neckControlMode;
   private final TDoubleArrayList neckDesiredJointAccelerations = new TDoubleArrayList(10);

   public NeckDesiredAccelerationsCommand()
   {
   }

   @Override
   public void clear()
   {
      neckControlMode = null;
      neckDesiredJointAccelerations.reset();
   }

   @Override
   public void set(NeckDesiredAccelerationsMessage message)
   {
      neckControlMode = message.getNeckControlMode();
      neckDesiredJointAccelerations.reset();
      for (int i = 0; i < message.getNumberOfJoints(); i++)
         neckDesiredJointAccelerations.add(message.getNeckDesiredJointAcceleration(i));
   }

   @Override
   public void set(NeckDesiredAccelerationsCommand other)
   {
      neckControlMode = other.neckControlMode;
      neckDesiredJointAccelerations.reset();
      for (int i = 0; i < other.getNumberOfJoints(); i++)
         neckDesiredJointAccelerations.add(other.getNeckDesiredJointAcceleration(i));
   }

   public int getNumberOfJoints()
   {
      return neckDesiredJointAccelerations.size();
   }

   public double getNeckDesiredJointAcceleration(int jointIndex)
   {
      return neckDesiredJointAccelerations.get(jointIndex);
   }

   public void setNeckControlMode(NeckControlMode neckControlMode)
   {
      this.neckControlMode = neckControlMode;
   }

   public NeckControlMode getNeckControlMode()
   {
      return neckControlMode;
   }

   public TDoubleArrayList getNeckDesiredJointAccelerations()
   {
      return neckDesiredJointAccelerations;
   }

   @Override
   public Class<NeckDesiredAccelerationsMessage> getMessageClass()
   {
      return NeckDesiredAccelerationsMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return neckControlMode != null && !neckDesiredJointAccelerations.isEmpty();
   }
}
