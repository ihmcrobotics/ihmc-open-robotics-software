package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.CenterOfMassTrajectoryMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameBasedCommand;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class CenterOfMassTrajectoryCommand implements Command<CenterOfMassTrajectoryCommand, CenterOfMassTrajectoryMessage>, FrameBasedCommand<CenterOfMassTrajectoryMessage>
{
   private long sequenceId;
   private final EuclideanTrajectoryControllerCommand euclideanTrajectory = new EuclideanTrajectoryControllerCommand();

   public CenterOfMassTrajectoryCommand()
   {
      euclideanTrajectory.clear();
   }

   public CenterOfMassTrajectoryCommand(CenterOfMassTrajectoryCommand other)
   {
      set(other);
   }

   @Override
   public void set(CenterOfMassTrajectoryCommand other)
   {
      sequenceId = other.sequenceId;
      euclideanTrajectory.set(other.euclideanTrajectory);
   }

   @Override
   public void setFromMessage(CenterOfMassTrajectoryMessage message)
   {
      FrameBasedCommand.super.setFromMessage(message);
   }

   @Override
   public void set(ReferenceFrameHashCodeResolver resolver, CenterOfMassTrajectoryMessage message)
   {
      sequenceId = message.getSequenceId();
      euclideanTrajectory.set(resolver, message.getEuclideanTrajectory());
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      euclideanTrajectory.clear();
   }

   public EuclideanTrajectoryControllerCommand getEuclideanTrajectory()
   {
      return euclideanTrajectory;
   }

   @Override
   public Class<CenterOfMassTrajectoryMessage> getMessageClass()
   {
      return CenterOfMassTrajectoryMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return euclideanTrajectory.isCommandValid();
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
