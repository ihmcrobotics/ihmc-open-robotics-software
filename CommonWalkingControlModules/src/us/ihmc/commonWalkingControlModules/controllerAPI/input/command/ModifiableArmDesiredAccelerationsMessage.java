package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmDesiredAccelerationsMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmDesiredAccelerationsMessage.ArmControlMode;
import us.ihmc.robotics.robotSide.RobotSide;

public class ModifiableArmDesiredAccelerationsMessage
      implements ControllerSideDependentMessage<ModifiableArmDesiredAccelerationsMessage, ArmDesiredAccelerationsMessage>
{
   private RobotSide robotSide;
   private ArmControlMode armControlMode;
   private final TDoubleArrayList armDesiredJointAccelerations = new TDoubleArrayList(10);

   public ModifiableArmDesiredAccelerationsMessage()
   {
   }

   @Override
   public void set(ArmDesiredAccelerationsMessage message)
   {
      robotSide = message.getRobotSide();
      armControlMode = message.getArmControlMode();
      armDesiredJointAccelerations.reset();
      for (int i = 0; i < message.getNumberOfJoints(); i++)
         armDesiredJointAccelerations.add(message.getArmDesiredJointAcceleration(i));
   }

   @Override
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

   @Override
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

   @Override
   public Class<ArmDesiredAccelerationsMessage> getMessageClass()
   {
      return ArmDesiredAccelerationsMessage.class;
   }
}
