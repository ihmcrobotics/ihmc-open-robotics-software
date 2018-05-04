package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.QuadrupedBodyOrientationMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameBasedCommand;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

import java.util.Random;

public class QuadrupedBodyOrientationCommand implements Command<QuadrupedBodyOrientationCommand, QuadrupedBodyOrientationMessage>,
      FrameBasedCommand<QuadrupedBodyOrientationMessage>, EpsilonComparable<QuadrupedBodyOrientationCommand>
{
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
      so3Trajectory = new SO3TrajectoryControllerCommand();
   }

   public QuadrupedBodyOrientationCommand(Random random)
   {
      so3Trajectory = new SO3TrajectoryControllerCommand(random);
      isExpressedInAbsoluteTime = random.nextBoolean();
      isAnOffsetOrientation = random.nextBoolean();
   }

   @Override
   public void clear()
   {
      isExpressedInAbsoluteTime = true;
      isAnOffsetOrientation = true;
      so3Trajectory.clear();
   }

   @Override
   public void set(QuadrupedBodyOrientationCommand other)
   {
      isExpressedInAbsoluteTime = other.isExpressedInAbsoluteTime;
      isAnOffsetOrientation = other.isAnOffsetOrientation;
      so3Trajectory.set(other.so3Trajectory);
   }

   @Override
   public void set(ReferenceFrameHashCodeResolver resolver, QuadrupedBodyOrientationMessage message)
   {
      isExpressedInAbsoluteTime = message.getIsExpressedInAbsoluteTime();
      isAnOffsetOrientation = message.getIsAnOffsetOrientation();
      so3Trajectory.set(resolver, message.getSo3Trajectory());
   }

   @Override
   public void set(QuadrupedBodyOrientationMessage message)
   {
      isExpressedInAbsoluteTime = message.getIsExpressedInAbsoluteTime();
      isAnOffsetOrientation = message.getIsAnOffsetOrientation();
      so3Trajectory.set(message.getSo3Trajectory());
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
}
