package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.Random;

import controller_msgs.msg.dds.HeadTrajectoryMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameBasedCommand;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class HeadTrajectoryCommand
      implements Command<HeadTrajectoryCommand, HeadTrajectoryMessage>, FrameBasedCommand<HeadTrajectoryMessage>, EpsilonComparable<HeadTrajectoryCommand>
{
   private long sequenceId;
   private final SO3TrajectoryControllerCommand so3Trajectory;

   public HeadTrajectoryCommand()
   {
      so3Trajectory = new SO3TrajectoryControllerCommand();
   }

   public HeadTrajectoryCommand(Random random)
   {
      so3Trajectory = new SO3TrajectoryControllerCommand(random);
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      so3Trajectory.clear();
   }

   @Override
   public void set(HeadTrajectoryCommand other)
   {
      sequenceId = other.sequenceId;
      so3Trajectory.set(other.so3Trajectory);
   }

   @Override
   public void setFromMessage(HeadTrajectoryMessage message)
   {
      FrameBasedCommand.super.setFromMessage(message);
   }

   @Override
   public void set(ReferenceFrameHashCodeResolver resolver, HeadTrajectoryMessage message)
   {
      sequenceId = message.getSequenceId();
      so3Trajectory.set(resolver, message.getSo3Trajectory());
   }

   @Override
   public boolean epsilonEquals(HeadTrajectoryCommand other, double epsilon)
   {
      return so3Trajectory.epsilonEquals(other.so3Trajectory, epsilon);
   }

   public SO3TrajectoryControllerCommand getSO3Trajectory()
   {
      return so3Trajectory;
   }

   @Override
   public boolean isCommandValid()
   {
      return so3Trajectory.isCommandValid();
   }

   @Override
   public Class<HeadTrajectoryMessage> getMessageClass()
   {
      return HeadTrajectoryMessage.class;
   }
   
   @Override
   public boolean isDelayedExecutionSupported()
   {
      return true;
   }

   @Override
   public void setExecutionDelayTime(double delayTime)
   {
      so3Trajectory.setExecutionDelayTime(delayTime);
   }

   @Override
   public void setExecutionTime(double adjustedExecutionTime)
   {
      so3Trajectory.setExecutionTime(adjustedExecutionTime);
   }

   @Override
   public double getExecutionDelayTime()
   {
      return so3Trajectory.getExecutionDelayTime();
   }

   @Override
   public double getExecutionTime()
   {
      return so3Trajectory.getExecutionTime();
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
