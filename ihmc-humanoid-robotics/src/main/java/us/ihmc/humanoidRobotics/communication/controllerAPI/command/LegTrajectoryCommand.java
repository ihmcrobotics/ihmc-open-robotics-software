package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.Random;

import controller_msgs.msg.dds.LegTrajectoryMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.robotics.robotSide.RobotSide;

public class LegTrajectoryCommand implements Command<LegTrajectoryCommand, LegTrajectoryMessage>, EpsilonComparable<LegTrajectoryCommand>
{
   private long sequenceId;
   private RobotSide robotSide;
   private final JointspaceTrajectoryCommand jointspaceTrajectory;

   public LegTrajectoryCommand()
   {
      sequenceId = 0;
      robotSide = null;
      jointspaceTrajectory = new JointspaceTrajectoryCommand();
   }

   public LegTrajectoryCommand(Random random)
   {
      sequenceId = random.nextInt();
      robotSide = random.nextBoolean() ? RobotSide.LEFT : RobotSide.RIGHT;
      jointspaceTrajectory = new JointspaceTrajectoryCommand(random);
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      robotSide = null;
      jointspaceTrajectory.clear();
   }

   public void clear(RobotSide robotSide)
   {
      clear();
      this.robotSide = robotSide;
   }

   @Override
   public void setFromMessage(LegTrajectoryMessage message)
   {
      clear(RobotSide.fromByte(message.getRobotSide()));
      sequenceId = message.getSequenceId();
      jointspaceTrajectory.setFromMessage(message.getJointspaceTrajectory());
   }

   @Override
   public void set(LegTrajectoryCommand other)
   {
      clear(other.getRobotSide());
      sequenceId = other.sequenceId;
      jointspaceTrajectory.set(other.getJointspaceTrajectory());
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
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
   public Class<LegTrajectoryMessage> getMessageClass()
   {
      return LegTrajectoryMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return robotSide != null && jointspaceTrajectory.isCommandValid();
   }

   @Override
   public boolean epsilonEquals(LegTrajectoryCommand other, double epsilon)
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

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
