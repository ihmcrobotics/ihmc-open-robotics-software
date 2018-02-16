package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.Random;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.robotics.robotSide.RobotSide;

public class ArmTrajectoryCommand implements Command<ArmTrajectoryCommand, ArmTrajectoryMessage>, EpsilonComparable<ArmTrajectoryCommand>
{
   private RobotSide robotSide;
   private final JointspaceTrajectoryCommand jointspaceTrajectory;

   public ArmTrajectoryCommand()
   {
      robotSide = null;
      jointspaceTrajectory = new JointspaceTrajectoryCommand();
   }

   public ArmTrajectoryCommand(Random random)
   {
      robotSide = random.nextBoolean() ? RobotSide.LEFT : RobotSide.RIGHT;
      jointspaceTrajectory = new JointspaceTrajectoryCommand(random);
   }

   @Override
   public void clear()
   {
      robotSide = null;
      jointspaceTrajectory.clear();
   }

   public void clear(RobotSide robotSide)
   {
      this.robotSide = robotSide;
      jointspaceTrajectory.clear();
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   @Override
   public void set(ArmTrajectoryMessage message)
   {
      clear(RobotSide.fromByte(message.getRobotSide()));
      jointspaceTrajectory.set(message.getJointspaceTrajectory());
   }

   @Override
   public void set(ArmTrajectoryCommand other)
   {
      clear(other.getRobotSide());
      jointspaceTrajectory.set(other.getJointspaceTrajectory());
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public JointspaceTrajectoryCommand getJointspaceTrajectory()
   {
      return jointspaceTrajectory;
   }

   @Override
   public Class<ArmTrajectoryMessage> getMessageClass()
   {
      return ArmTrajectoryMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return robotSide != null && jointspaceTrajectory.isCommandValid();
   }

   @Override
   public boolean epsilonEquals(ArmTrajectoryCommand other, double epsilon)
   {
      if (robotSide != other.robotSide)
         return false;
      if (!jointspaceTrajectory.epsilonEquals(other.jointspaceTrajectory, epsilon))
         return false;
      return true;
   }

   @Override
   public boolean isDelayedExecutionSupported()
   {
      return true;
   }

   @Override
   public void setExecutionDelayTime(double delayTime)
   {
      jointspaceTrajectory.setExecutionDelayTime(delayTime);
   }

   @Override
   public void setExecutionTime(double adjustedExecutionTime)
   {
      jointspaceTrajectory.setExecutionTime(adjustedExecutionTime);
   }

   @Override
   public double getExecutionDelayTime()
   {
      return jointspaceTrajectory.getExecutionDelayTime();
   }

   @Override
   public double getExecutionTime()
   {
      return jointspaceTrajectory.getExecutionTime();
   }
}
