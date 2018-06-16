package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.Random;

import controller_msgs.msg.dds.ChestHybridJointspaceTaskspaceTrajectoryMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameBasedCommand;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class ChestHybridJointspaceTaskspaceTrajectoryCommand
      implements Command<ChestHybridJointspaceTaskspaceTrajectoryCommand, ChestHybridJointspaceTaskspaceTrajectoryMessage>,
      FrameBasedCommand<ChestHybridJointspaceTaskspaceTrajectoryMessage>
{
   private final JointspaceTrajectoryCommand jointspaceTrajectoryCommand = new JointspaceTrajectoryCommand();
   private final SE3TrajectoryControllerCommand taskspaceTrajectoryCommand = new SE3TrajectoryControllerCommand();

   public ChestHybridJointspaceTaskspaceTrajectoryCommand()
   {
   }

   public ChestHybridJointspaceTaskspaceTrajectoryCommand(SO3TrajectoryControllerCommand taskspaceTrajectoryCommand,
                                                          JointspaceTrajectoryCommand jointspaceTrajectoryCommand)
   {
      super();
      this.jointspaceTrajectoryCommand.set(jointspaceTrajectoryCommand);
      this.taskspaceTrajectoryCommand.setToOrientationTrajectory(taskspaceTrajectoryCommand);
   }

   public ChestHybridJointspaceTaskspaceTrajectoryCommand(Random random)
   {
      this(new SO3TrajectoryControllerCommand(random), new JointspaceTrajectoryCommand(random));
   }

   @Override
   public void clear()
   {
      jointspaceTrajectoryCommand.clear();
      taskspaceTrajectoryCommand.clear();
   }

   @Override
   public void setFromMessage(ChestHybridJointspaceTaskspaceTrajectoryMessage message)
   {
      jointspaceTrajectoryCommand.setFromMessage(message.getJointspaceTrajectoryMessage());
      taskspaceTrajectoryCommand.setToOrientationTrajectory(message.getTaskspaceTrajectoryMessage());
   }

   @Override
   public void set(ReferenceFrameHashCodeResolver resolver, ChestHybridJointspaceTaskspaceTrajectoryMessage message)
   {
      jointspaceTrajectoryCommand.setFromMessage(message.getJointspaceTrajectoryMessage());
      taskspaceTrajectoryCommand.setToOrientationTrajectory(resolver, message.getTaskspaceTrajectoryMessage());
   }

   @Override
   public boolean isCommandValid()
   {
      return jointspaceTrajectoryCommand.isCommandValid() && taskspaceTrajectoryCommand.isCommandValid();
   }

   @Override
   public void set(ChestHybridJointspaceTaskspaceTrajectoryCommand other)
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
   public Class<ChestHybridJointspaceTaskspaceTrajectoryMessage> getMessageClass()
   {
      return ChestHybridJointspaceTaskspaceTrajectoryMessage.class;
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
