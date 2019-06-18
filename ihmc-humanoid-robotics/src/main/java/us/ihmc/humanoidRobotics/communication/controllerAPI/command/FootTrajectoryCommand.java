package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.Random;

import controller_msgs.msg.dds.FootTrajectoryMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameBasedCommand;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class FootTrajectoryCommand implements Command<FootTrajectoryCommand, FootTrajectoryMessage>, FrameBasedCommand<FootTrajectoryMessage>, EpsilonComparable<FootTrajectoryCommand>
{
   private long sequenceId;
   private RobotSide robotSide;
   private final SE3TrajectoryControllerCommand se3Trajectory;

   public FootTrajectoryCommand()
   {
      se3Trajectory = new SE3TrajectoryControllerCommand();
   }

   public FootTrajectoryCommand(Random random)
   {
      se3Trajectory = new SE3TrajectoryControllerCommand(random);
      robotSide = RobotSide.generateRandomRobotSide(random);
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      se3Trajectory.clear();
      robotSide = null;
   }

   @Override
   public void setFromMessage(FootTrajectoryMessage message)
   {
      FrameBasedCommand.super.setFromMessage(message);
   }

   @Override
   public void set(ReferenceFrameHashCodeResolver resolver, FootTrajectoryMessage message)
   {
      sequenceId = message.getSequenceId();
      se3Trajectory.set(resolver, message.getSe3Trajectory());
      robotSide = RobotSide.fromByte(message.getRobotSide());
   }

   @Override
   public void set(FootTrajectoryCommand other)
   {
      se3Trajectory.set(other.se3Trajectory);
      setPropertiesOnly(other);
   }

   /**
    * Same as {@link #set(FootTrajectoryCommand)} but does not change the trajectory points.
    *
    * @param other
    */
   public void setPropertiesOnly(FootTrajectoryCommand other)
   {
      sequenceId = other.sequenceId;
      se3Trajectory.setPropertiesOnly(other.se3Trajectory);
      robotSide = other.robotSide;
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public SE3TrajectoryControllerCommand getSE3Trajectory()
   {
      return se3Trajectory;
   }

   @Override
   public Class<FootTrajectoryMessage> getMessageClass()
   {
      return FootTrajectoryMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return robotSide != null && se3Trajectory.isCommandValid();
   }

   @Override
   public boolean epsilonEquals(FootTrajectoryCommand other, double epsilon)
   {
      return robotSide == other.robotSide && se3Trajectory.epsilonEquals(other.se3Trajectory, epsilon);
   }
   
   @Override
   public boolean isDelayedExecutionSupported()
   {
      return true;
   }

   @Override
   public void setExecutionDelayTime(double delayTime)
   {
      se3Trajectory.setExecutionDelayTime(delayTime);
   }

   @Override
   public void setExecutionTime(double adjustedExecutionTime)
   {
      se3Trajectory.setExecutionTime(adjustedExecutionTime);
   }

   @Override
   public double getExecutionDelayTime()
   {
      return se3Trajectory.getExecutionDelayTime();
   }

   @Override
   public double getExecutionTime()
   {
      return se3Trajectory.getExecutionTime();
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
