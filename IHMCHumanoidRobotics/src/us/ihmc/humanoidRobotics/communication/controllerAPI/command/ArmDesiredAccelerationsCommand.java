package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmDesiredAccelerationsMessage;
import us.ihmc.robotics.robotSide.RobotSide;

public class ArmDesiredAccelerationsCommand
      implements Command<ArmDesiredAccelerationsCommand, ArmDesiredAccelerationsMessage>
{
   private RobotSide robotSide;
   private final TDoubleArrayList armDesiredJointAccelerations = new TDoubleArrayList(10);

   public ArmDesiredAccelerationsCommand()
   {
   }

   @Override
   public void clear()
   {
      robotSide = null;
      armDesiredJointAccelerations.reset();
   }

   @Override
   public void set(ArmDesiredAccelerationsMessage message)
   {
      robotSide = message.getRobotSide();
      armDesiredJointAccelerations.reset();
      for (int i = 0; i < message.getNumberOfJoints(); i++)
         armDesiredJointAccelerations.add(message.getDesiredJointAcceleration(i));
   }

   @Override
   public void set(ArmDesiredAccelerationsCommand other)
   {
      robotSide = other.robotSide;
      armDesiredJointAccelerations.reset();
      for (int i = 0; i < other.getNumberOfJoints(); i++)
         armDesiredJointAccelerations.add(other.getArmDesiredJointAcceleration(i));
   }

   public int getNumberOfJoints()
   {
      return armDesiredJointAccelerations.size();
   }

   public double getArmDesiredJointAcceleration(int jointIndex)
   {
      return armDesiredJointAccelerations.get(jointIndex);
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public TDoubleArrayList getArmDesiredJointAccelerations()
   {
      return armDesiredJointAccelerations;
   }

   @Override
   public Class<ArmDesiredAccelerationsMessage> getMessageClass()
   {
      return ArmDesiredAccelerationsMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return robotSide != null && !armDesiredJointAccelerations.isEmpty();
   }
}
