package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.Random;

import controller_msgs.msg.dds.PelvisOrientationTrajectoryMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameBasedCommand;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class PelvisOrientationTrajectoryCommand implements Command<PelvisOrientationTrajectoryCommand, PelvisOrientationTrajectoryMessage>,
      FrameBasedCommand<PelvisOrientationTrajectoryMessage>, EpsilonComparable<PelvisOrientationTrajectoryCommand>
{
   private boolean enableUserPelvisControlDuringWalking = false;
   private final SO3TrajectoryControllerCommand so3Trajectory;

   public PelvisOrientationTrajectoryCommand()
   {
      so3Trajectory = new SO3TrajectoryControllerCommand();
   }

   public PelvisOrientationTrajectoryCommand(Random random)
   {
      so3Trajectory = new SO3TrajectoryControllerCommand(random);
   }

   @Override
   public void clear()
   {
      so3Trajectory.clear();
   }

   @Override
   public void set(PelvisOrientationTrajectoryCommand other)
   {
      setEnableUserPelvisControlDuringWalking(other.isEnableUserPelvisControlDuringWalking());
      so3Trajectory.set(other.so3Trajectory);
   }

   @Override
   public void set(ReferenceFrameHashCodeResolver resolver, PelvisOrientationTrajectoryMessage message)
   {
      setEnableUserPelvisControlDuringWalking(message.getEnableUserPelvisControlDuringWalking());
      so3Trajectory.set(resolver, message.getSo3Trajectory());
   }

   @Override
   public void set(PelvisOrientationTrajectoryMessage message)
   {
      setEnableUserPelvisControlDuringWalking(message.getEnableUserPelvisControlDuringWalking());
      so3Trajectory.set(message.getSo3Trajectory());
   }

   /**
    * Allows setting this orientation {@link #SO3TrajectoryControllerCommand} trajectory command
    * from a pelvis pose {@link #SE3TrajectoryControllerCommand} trajectory command.
    */
   public void set(PelvisTrajectoryCommand command)
   {
      setEnableUserPelvisControlDuringWalking(command.isEnableUserPelvisControlDuringWalking());
      so3Trajectory.set(command.getSE3Trajectory());
   }

   public boolean isEnableUserPelvisControlDuringWalking()
   {
      return enableUserPelvisControlDuringWalking;
   }

   public void setEnableUserPelvisControlDuringWalking(boolean enableUserPelvisControlDuringWalking)
   {
      this.enableUserPelvisControlDuringWalking = enableUserPelvisControlDuringWalking;
   }

   @Override
   public boolean epsilonEquals(PelvisOrientationTrajectoryCommand other, double epsilon)
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
   public Class<PelvisOrientationTrajectoryMessage> getMessageClass()
   {
      return PelvisOrientationTrajectoryMessage.class;
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
