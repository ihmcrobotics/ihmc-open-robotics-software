package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.Random;

import controller_msgs.msg.dds.HeadHybridJointspaceTaskspaceTrajectoryMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameBasedCommand;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class HeadHybridJointspaceTaskspaceTrajectoryCommand
      implements Command<HeadHybridJointspaceTaskspaceTrajectoryCommand, HeadHybridJointspaceTaskspaceTrajectoryMessage>,
      FrameBasedCommand<HeadHybridJointspaceTaskspaceTrajectoryMessage>
{
   private long sequenceId;
   private final JointspaceTrajectoryCommand jointspaceTrajectoryCommand = new JointspaceTrajectoryCommand();
   private final SO3TrajectoryControllerCommand taskspaceTrajectoryCommand = new SO3TrajectoryControllerCommand();

   public HeadHybridJointspaceTaskspaceTrajectoryCommand()
   {
   }

   public HeadHybridJointspaceTaskspaceTrajectoryCommand(SO3TrajectoryControllerCommand taskspaceTrajectoryCommand,
                                                         JointspaceTrajectoryCommand jointspaceTrajectoryCommand)
   {
      this.jointspaceTrajectoryCommand.set(jointspaceTrajectoryCommand);
      this.taskspaceTrajectoryCommand.set(taskspaceTrajectoryCommand);
   }

   public HeadHybridJointspaceTaskspaceTrajectoryCommand(Random random)
   {
      this(new SO3TrajectoryControllerCommand(random), new JointspaceTrajectoryCommand(random));
   }

   @Override
   public Class<HeadHybridJointspaceTaskspaceTrajectoryMessage> getMessageClass()
   {
      return HeadHybridJointspaceTaskspaceTrajectoryMessage.class;
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      jointspaceTrajectoryCommand.clear();
      taskspaceTrajectoryCommand.clear();
   }

   @Override
   public void setFromMessage(HeadHybridJointspaceTaskspaceTrajectoryMessage message)
   {
      FrameBasedCommand.super.setFromMessage(message);
   }

   @Override
   public void set(ReferenceFrameHashCodeResolver resolver, HeadHybridJointspaceTaskspaceTrajectoryMessage message)
   {
      sequenceId = message.getSequenceId();
      jointspaceTrajectoryCommand.setFromMessage(message.getJointspaceTrajectoryMessage());
      taskspaceTrajectoryCommand.set(resolver, message.getTaskspaceTrajectoryMessage());
   }

   @Override
   public boolean isCommandValid()
   {
      return jointspaceTrajectoryCommand.isCommandValid() && taskspaceTrajectoryCommand.isCommandValid();
   }

   @Override
   public void set(HeadHybridJointspaceTaskspaceTrajectoryCommand other)
   {
      sequenceId = other.sequenceId;
      taskspaceTrajectoryCommand.set(other.getTaskspaceTrajectoryCommand());
      jointspaceTrajectoryCommand.set(other.getJointspaceTrajectoryCommand());
   }

   public JointspaceTrajectoryCommand getJointspaceTrajectoryCommand()
   {
      return jointspaceTrajectoryCommand;
   }

   public SO3TrajectoryControllerCommand getTaskspaceTrajectoryCommand()
   {
      return taskspaceTrajectoryCommand;
   }

   @Override
   public boolean isDelayedExecutionSupported()
   {
      return true;
   }

   @Override
   public void setExecutionDelayTime(double delayTime)
   {
      taskspaceTrajectoryCommand.setExecutionDelayTime(delayTime);
   }

   @Override
   public void setExecutionTime(double adjustedExecutionTime)
   {
      taskspaceTrajectoryCommand.setExecutionTime(adjustedExecutionTime);
   }

   @Override
   public double getExecutionDelayTime()
   {
      return taskspaceTrajectoryCommand.getExecutionDelayTime();
   }

   @Override
   public double getExecutionTime()
   {
      return taskspaceTrajectoryCommand.getExecutionTime();
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
