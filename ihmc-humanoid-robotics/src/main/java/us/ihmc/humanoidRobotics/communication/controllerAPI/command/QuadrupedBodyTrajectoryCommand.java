package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.QuadrupedBodyTrajectoryMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameBasedCommand;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

import java.util.Random;

public class QuadrupedBodyTrajectoryCommand
      implements Command<QuadrupedBodyTrajectoryCommand, QuadrupedBodyTrajectoryMessage>, FrameBasedCommand<QuadrupedBodyTrajectoryMessage>, EpsilonComparable<QuadrupedBodyTrajectoryCommand>
{
   private long sequenceId;
   private boolean isExpressedInAbsoluteTime;

   /**
    * Desired body pose trajectory, expressed in world frame
    */
   private final SE3TrajectoryControllerCommand se3Trajectory;

   public QuadrupedBodyTrajectoryCommand()
   {
      sequenceId = 0;
      se3Trajectory = new SE3TrajectoryControllerCommand();
   }

   public QuadrupedBodyTrajectoryCommand(Random random)
   {
      sequenceId = random.nextInt();
      se3Trajectory = new SE3TrajectoryControllerCommand(random);
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      isExpressedInAbsoluteTime = true;
      se3Trajectory.clear();
   }

   @Override
   public void set(QuadrupedBodyTrajectoryCommand other)
   {
      sequenceId = other.sequenceId;
      isExpressedInAbsoluteTime = other.isExpressedInAbsoluteTime;
      se3Trajectory.set(other.se3Trajectory);
   }

   @Override
   public void setFromMessage(QuadrupedBodyTrajectoryMessage message)
   {
      FrameBasedCommand.super.setFromMessage(message);
   }

   @Override
   public void set(ReferenceFrameHashCodeResolver resolver, QuadrupedBodyTrajectoryMessage message)
   {
      sequenceId = message.getSequenceId();
      isExpressedInAbsoluteTime = message.getIsExpressedInAbsoluteTime();
      se3Trajectory.set(resolver, message.getSe3Trajectory());
   }

   public boolean isExpressedInAbsoluteTime()
   {
      return isExpressedInAbsoluteTime;
   }

   public void setIsExpressedInAbsoluteTime(boolean isExpressedInAbsoluteTime)
   {
      this.isExpressedInAbsoluteTime = isExpressedInAbsoluteTime;
   }

   public SE3TrajectoryControllerCommand getSE3Trajectory()
   {
      return se3Trajectory;
   }

   @Override
   public Class<QuadrupedBodyTrajectoryMessage> getMessageClass()
   {
      return QuadrupedBodyTrajectoryMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return se3Trajectory.isCommandValid();
   }

   @Override
   public boolean epsilonEquals(QuadrupedBodyTrajectoryCommand other, double epsilon)
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
