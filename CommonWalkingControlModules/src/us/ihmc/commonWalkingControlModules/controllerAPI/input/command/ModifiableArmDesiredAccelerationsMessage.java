package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmDesiredAccelerationsMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmDesiredAccelerationsMessage.ArmControlMode;
import us.ihmc.robotics.robotSide.RobotSide;

public class ModifiableArmDesiredAccelerationsMessage
{
   private RobotSide robotSide;
   private ArmControlMode armControlMode;
   private final TDoubleArrayList armDesiredJointAccelerations = new TDoubleArrayList(10);

   public ModifiableArmDesiredAccelerationsMessage()
   {
   }

   public void set(ArmDesiredAccelerationsMessage armDesiredAccelerationsMessage)
   {
      robotSide = armDesiredAccelerationsMessage.getRobotSide();
      armControlMode = armDesiredAccelerationsMessage.getArmControlMode();
      armDesiredJointAccelerations.reset();
      for (int i = 0; i < armDesiredAccelerationsMessage.getNumberOfJoints(); i++)
         armDesiredJointAccelerations.add(armDesiredAccelerationsMessage.getArmDesiredJointAcceleration(i));
   }

   public void set(ModifiableArmDesiredAccelerationsMessage other)
   {
      robotSide = other.robotSide;
      armControlMode = other.armControlMode;
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

   public void setArmControlMode(ArmControlMode armControlMode)
   {
      this.armControlMode = armControlMode;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public ArmControlMode getArmControlMode()
   {
      return armControlMode;
   }

   public TDoubleArrayList getArmDesiredJointAccelerations()
   {
      return armDesiredJointAccelerations;
   }
}
