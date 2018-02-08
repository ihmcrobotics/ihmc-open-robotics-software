package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.Random;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameBasedCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class ChestTrajectoryCommand
      implements Command<ChestTrajectoryCommand, ChestTrajectoryMessage>, FrameBasedCommand<ChestTrajectoryMessage>, EpsilonComparable<ChestTrajectoryCommand>
{
   private final SO3TrajectoryControllerCommand so3Trajectory;

   public ChestTrajectoryCommand()
   {
      so3Trajectory = new SO3TrajectoryControllerCommand();
   }

   public ChestTrajectoryCommand(Random random)
   {
      so3Trajectory = new SO3TrajectoryControllerCommand(random);
   }

   @Override
   public void clear()
   {
      so3Trajectory.clear();
   }

   @Override
   public void set(ChestTrajectoryCommand other)
   {
      so3Trajectory.set(other.so3Trajectory);
   }

   @Override
   public void set(ChestTrajectoryMessage message)
   {
      so3Trajectory.set(message.so3Trajectory);
   }

   @Override
   public void set(ReferenceFrameHashCodeResolver resolver, ChestTrajectoryMessage message)
   {
      so3Trajectory.set(resolver, message.so3Trajectory);
   }

   @Override
   public boolean epsilonEquals(ChestTrajectoryCommand other, double epsilon)
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
   public Class<ChestTrajectoryMessage> getMessageClass()
   {
      return ChestTrajectoryMessage.class;
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
}
