package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.QuadrupedBodyHeightMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameBasedCommand;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class QuadrupedBodyHeightCommand
      implements Command<QuadrupedBodyHeightCommand, QuadrupedBodyHeightMessage>, FrameBasedCommand<QuadrupedBodyHeightMessage>,
      EpsilonComparable<QuadrupedBodyHeightCommand>
{
   private long sequenceId;
   /**
    * This trajectory is sent to control the body height
    */
   private boolean controlBodyHeight = false;

   /**
    * This trajectory is sent in absolute time, expressed relative to the robot start
    */
   private boolean isExpressedInAbsoluteTime = false;

   private final EuclideanTrajectoryControllerCommand euclideanTrajectory;

   public QuadrupedBodyHeightCommand()
   {
      euclideanTrajectory = new EuclideanTrajectoryControllerCommand();
   }

   public void clear()
   {
      sequenceId = 0;
      euclideanTrajectory.clear();
      controlBodyHeight = false;
      isExpressedInAbsoluteTime = false;
   }

   public void clear(ReferenceFrame referenceFrame)
   {
      euclideanTrajectory.clear(referenceFrame);
      controlBodyHeight = false;
      isExpressedInAbsoluteTime = false;
   }

   @Override
   public void set(QuadrupedBodyHeightCommand other)
   {
      sequenceId = other.sequenceId;
      euclideanTrajectory.set(other.euclideanTrajectory);
      controlBodyHeight = other.controlBodyHeight;
      isExpressedInAbsoluteTime = other.isExpressedInAbsoluteTime;
   }

   @Override
   public void setFromMessage(QuadrupedBodyHeightMessage message)
   {
      FrameBasedCommand.super.setFromMessage(message);
   }

   @Override
   public void set(ReferenceFrameHashCodeResolver resolver, QuadrupedBodyHeightMessage message)
   {
      sequenceId = message.getSequenceId();
      euclideanTrajectory.set(resolver, message.getEuclideanTrajectory());
      controlBodyHeight = message.getControlBodyHeight();
      isExpressedInAbsoluteTime = message.getIsExpressedInAbsoluteTime();
   }

   public boolean controlBodyHeight()
   {
      return controlBodyHeight;
   }

   public boolean isExpressedInAbsoluteTime()
   {
      return isExpressedInAbsoluteTime;
   }

   public EuclideanTrajectoryControllerCommand getEuclideanTrajectory()
   {
      return euclideanTrajectory;
   }

   @Override
   public Class<QuadrupedBodyHeightMessage> getMessageClass()
   {
      return QuadrupedBodyHeightMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return euclideanTrajectory.isCommandValid();
   }

   @Override
   public boolean epsilonEquals(QuadrupedBodyHeightCommand other, double epsilon)
   {
      if (isExpressedInAbsoluteTime != other.isExpressedInAbsoluteTime)
         return false;
      if (controlBodyHeight != other.controlBodyHeight)
         return false;

      return euclideanTrajectory.epsilonEquals(other.euclideanTrajectory, epsilon);
   }

   @Override
   public boolean isDelayedExecutionSupported()
   {
      return true;
   }

   @Override
   public void setExecutionDelayTime(double delayTime)
   {
      euclideanTrajectory.setExecutionDelayTime(delayTime);
   }

   @Override
   public void setExecutionTime(double adjustedExecutionTime)
   {
      euclideanTrajectory.setExecutionTime(adjustedExecutionTime);
   }

   @Override
   public double getExecutionDelayTime()
   {
      return euclideanTrajectory.getExecutionDelayTime();
   }

   @Override
   public double getExecutionTime()
   {
      return euclideanTrajectory.getExecutionTime();
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
