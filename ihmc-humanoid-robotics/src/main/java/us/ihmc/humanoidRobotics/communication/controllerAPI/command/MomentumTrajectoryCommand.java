package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.MomentumTrajectoryMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameBasedCommand;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class MomentumTrajectoryCommand implements Command<MomentumTrajectoryCommand, MomentumTrajectoryMessage>, FrameBasedCommand<MomentumTrajectoryMessage>
{
   private long sequenceId;
   private final EuclideanTrajectoryControllerCommand angularMomentumTrajectory = new EuclideanTrajectoryControllerCommand();

   public MomentumTrajectoryCommand()
   {
      angularMomentumTrajectory.clear();
   }

   public MomentumTrajectoryCommand(MomentumTrajectoryCommand other)
   {
      set(other);
   }

   @Override
   public void setFromMessage(MomentumTrajectoryMessage message)
   {
      FrameBasedCommand.super.setFromMessage(message);
   }

   @Override
   public void set(ReferenceFrameHashCodeResolver resolver, MomentumTrajectoryMessage message)
   {
      sequenceId = message.getSequenceId();
      angularMomentumTrajectory.set(resolver, message.getAngularMomentumTrajectory());
   }

   @Override
   public void set(MomentumTrajectoryCommand other)
   {
      sequenceId = other.sequenceId;
      angularMomentumTrajectory.set(other.angularMomentumTrajectory);
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      angularMomentumTrajectory.clear();
   }

   public EuclideanTrajectoryControllerCommand getAngularMomentumTrajectory()
   {
      return angularMomentumTrajectory;
   }

   @Override
   public Class<MomentumTrajectoryMessage> getMessageClass()
   {
      return MomentumTrajectoryMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return angularMomentumTrajectory.isCommandValid();
   }

   @Override
   public boolean isDelayedExecutionSupported()
   {
      return true;
   }

   @Override
   public void setExecutionDelayTime(double delayTime)
   {
      angularMomentumTrajectory.setExecutionDelayTime(delayTime);
   }

   @Override
   public void setExecutionTime(double adjustedExecutionTime)
   {
      angularMomentumTrajectory.setExecutionTime(adjustedExecutionTime);
   }

   @Override
   public double getExecutionDelayTime()
   {
      return angularMomentumTrajectory.getExecutionDelayTime();
   }

   @Override
   public double getExecutionTime()
   {
      return angularMomentumTrajectory.getExecutionTime();
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
