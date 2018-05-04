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
   private final JointspaceTrajectoryCommand jointspaceTrajectoryCommand = new JointspaceTrajectoryCommand();
   private final SE3TrajectoryControllerCommand taskspaceTrajectoryCommand = new SE3TrajectoryControllerCommand();

   public HeadHybridJointspaceTaskspaceTrajectoryCommand()
   {
   }

   public HeadHybridJointspaceTaskspaceTrajectoryCommand(SO3TrajectoryControllerCommand taskspaceTrajectoryCommand,
                                                         JointspaceTrajectoryCommand jointspaceTrajectoryCommand)
   {
      this.jointspaceTrajectoryCommand.set(jointspaceTrajectoryCommand);
      this.taskspaceTrajectoryCommand.setToOrientationTrajectory(taskspaceTrajectoryCommand);
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
      jointspaceTrajectoryCommand.clear();
      taskspaceTrajectoryCommand.clear();
   }

   @Override
   public void set(HeadHybridJointspaceTaskspaceTrajectoryMessage message)
   {
      jointspaceTrajectoryCommand.set(message.getJointspaceTrajectoryMessage());
      taskspaceTrajectoryCommand.setToOrientationTrajectory(message.getTaskspaceTrajectoryMessage());
   }

   @Override
   public void set(ReferenceFrameHashCodeResolver resolver, HeadHybridJointspaceTaskspaceTrajectoryMessage message)
   {
      jointspaceTrajectoryCommand.set(message.getJointspaceTrajectoryMessage());
      taskspaceTrajectoryCommand.setToOrientationTrajectory(resolver, message.getTaskspaceTrajectoryMessage());
   }

   @Override
   public boolean isCommandValid()
   {
      return jointspaceTrajectoryCommand.isCommandValid() && taskspaceTrajectoryCommand.isCommandValid();
   }

   @Override
   public void set(HeadHybridJointspaceTaskspaceTrajectoryCommand other)
   {
      taskspaceTrajectoryCommand.set(other.getTaskspaceTrajectoryCommand());
      jointspaceTrajectoryCommand.set(other.getJointspaceTrajectoryCommand());
   }

   public JointspaceTrajectoryCommand getJointspaceTrajectoryCommand()
   {
      return jointspaceTrajectoryCommand;
   }

   public SE3TrajectoryControllerCommand getTaskspaceTrajectoryCommand()
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
}
