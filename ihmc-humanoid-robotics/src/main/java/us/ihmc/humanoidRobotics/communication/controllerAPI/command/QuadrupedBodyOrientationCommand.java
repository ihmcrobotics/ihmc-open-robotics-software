package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.QuadrupedBodyOrientationMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.CommandConversionTools;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameBasedCommand;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

import java.util.Random;

public class QuadrupedBodyOrientationCommand implements Command<QuadrupedBodyOrientationCommand, QuadrupedBodyOrientationMessage>,
      FrameBasedCommand<QuadrupedBodyOrientationMessage>, EpsilonComparable<QuadrupedBodyOrientationCommand>
{
   private long sequenceId;
   private boolean isExpressedInAbsoluteTime;

   /**
    * Desired body orientation trajectory, expressed in world frame
    */
   private final SO3TrajectoryControllerCommand so3Trajectory;

   /**
    * Indicates if the given trajectory should be considered an "absolute" orientation or an "offset" orientation
    */
   private boolean isAnOffsetOrientation;

   public QuadrupedBodyOrientationCommand()
   {
      sequenceId = 0;
      so3Trajectory = new SO3TrajectoryControllerCommand();
   }

   public QuadrupedBodyOrientationCommand(Random random)
   {
      sequenceId = random.nextInt();
      so3Trajectory = new SO3TrajectoryControllerCommand(random);
      isExpressedInAbsoluteTime = random.nextBoolean();
      isAnOffsetOrientation = random.nextBoolean();
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      isExpressedInAbsoluteTime = true;
      isAnOffsetOrientation = true;
      so3Trajectory.clear();
   }

   @Override
   public void set(QuadrupedBodyOrientationCommand other)
   {
      sequenceId = other.sequenceId;
      isExpressedInAbsoluteTime = other.isExpressedInAbsoluteTime;
      isAnOffsetOrientation = other.isAnOffsetOrientation;
      so3Trajectory.set(other.so3Trajectory);
   }

   @Override
   public void setFromMessage(QuadrupedBodyOrientationMessage message)
   {
      FrameBasedCommand.super.setFromMessage(message);
   }

   @Override
   public void set(ReferenceFrameHashCodeResolver resolver, QuadrupedBodyOrientationMessage message)
   {
      sequenceId = message.getSequenceId();
      isExpressedInAbsoluteTime = message.getIsExpressedInAbsoluteTime();
      isAnOffsetOrientation = message.getIsAnOffsetOrientation();
      so3Trajectory.set(resolver, message.getSo3Trajectory());
   }

   public void set(QuadrupedBodyTrajectoryCommand command)
   {
      isExpressedInAbsoluteTime = command.isExpressedInAbsoluteTime();
      isAnOffsetOrientation = true;
      CommandConversionTools.convertToSO3(command.getSE3Trajectory(), so3Trajectory);
   }

   @Override
   public boolean epsilonEquals(QuadrupedBodyOrientationCommand other, double epsilon)
   {
      return isExpressedInAbsoluteTime == other.isExpressedInAbsoluteTime && isAnOffsetOrientation == other.isAnOffsetOrientation && so3Trajectory
            .epsilonEquals(other.so3Trajectory, epsilon);
   }

   public boolean isExpressedInAbsoluteTime()
   {
      return isExpressedInAbsoluteTime;
   }

   public boolean isAnOffsetOrientation()
   {
      return isAnOffsetOrientation;
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
   public Class<QuadrupedBodyOrientationMessage> getMessageClass()
   {
      return QuadrupedBodyOrientationMessage.class;
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
