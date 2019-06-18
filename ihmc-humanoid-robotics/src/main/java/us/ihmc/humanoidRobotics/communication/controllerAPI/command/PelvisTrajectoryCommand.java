package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.Random;

import controller_msgs.msg.dds.PelvisTrajectoryMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameBasedCommand;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class PelvisTrajectoryCommand implements Command<PelvisTrajectoryCommand, PelvisTrajectoryMessage>, FrameBasedCommand<PelvisTrajectoryMessage>, EpsilonComparable<PelvisTrajectoryCommand>
{
   private long sequenceId;
   private boolean enableUserPelvisControl = false;
   private boolean enableUserPelvisControlDuringWalking = false;
   private final SE3TrajectoryControllerCommand se3Trajectory;

   public PelvisTrajectoryCommand()
   {
      se3Trajectory = new SE3TrajectoryControllerCommand();
   }

   public PelvisTrajectoryCommand(Random random)
   {
      se3Trajectory = new SE3TrajectoryControllerCommand(random);
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      se3Trajectory.clear();
   }

   @Override
   public void set(PelvisTrajectoryCommand other)
   {
      sequenceId = other.sequenceId;
      setEnableUserPelvisControlDuringWalking(other.isEnableUserPelvisControlDuringWalking());
      setEnableUserPelvisControl(other.isEnableUserPelvisControl());
      se3Trajectory.set(other.se3Trajectory);
   }

   @Override
   public void setFromMessage(PelvisTrajectoryMessage message)
   {
      FrameBasedCommand.super.setFromMessage(message);
   }

   @Override
   public void set(ReferenceFrameHashCodeResolver resolver, PelvisTrajectoryMessage message)
   {
      sequenceId = message.getSequenceId();
      setEnableUserPelvisControlDuringWalking(message.getEnableUserPelvisControlDuringWalking());
      setEnableUserPelvisControl(message.getEnableUserPelvisControl());
      se3Trajectory.set(resolver, message.getSe3Trajectory());
   }

   public boolean isEnableUserPelvisControlDuringWalking()
   {
      return enableUserPelvisControlDuringWalking;
   }

   public void setEnableUserPelvisControlDuringWalking(boolean enableUserPelvisControlDuringWalking)
   {
      this.enableUserPelvisControlDuringWalking = enableUserPelvisControlDuringWalking;
   }

   public boolean isEnableUserPelvisControl()
   {
      return enableUserPelvisControl;
   }

   public void setEnableUserPelvisControl(boolean enableUserPelvisControl)
   {
      this.enableUserPelvisControl = enableUserPelvisControl;
   }

   public SE3TrajectoryControllerCommand getSE3Trajectory()
   {
      return se3Trajectory;
   }

   @Override
   public Class<PelvisTrajectoryMessage> getMessageClass()
   {
      return PelvisTrajectoryMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return se3Trajectory.isCommandValid();
   }

   @Override
   public boolean epsilonEquals(PelvisTrajectoryCommand other, double epsilon)
   {
      return se3Trajectory.epsilonEquals(other.se3Trajectory, epsilon);
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
