package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.Random;

import controller_msgs.msg.dds.HandHybridJointspaceTaskspaceTrajectoryMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameBasedCommand;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class HandHybridJointspaceTaskspaceTrajectoryCommand
      implements Command<HandHybridJointspaceTaskspaceTrajectoryCommand, HandHybridJointspaceTaskspaceTrajectoryMessage>,
      FrameBasedCommand<HandHybridJointspaceTaskspaceTrajectoryMessage>
{
   private long sequenceId;
   private RobotSide robotSide;
   private final JointspaceTrajectoryCommand jointspaceTrajectoryCommand = new JointspaceTrajectoryCommand();
   private final SE3TrajectoryControllerCommand taskspaceTrajectoryCommand = new SE3TrajectoryControllerCommand();

   public HandHybridJointspaceTaskspaceTrajectoryCommand()
   {
   }

   public HandHybridJointspaceTaskspaceTrajectoryCommand(RobotSide robotSide, SE3TrajectoryControllerCommand taskspaceTrajectoryCommand,
                                                         JointspaceTrajectoryCommand jointspaceTrajectoryCommand)
   {
      this.robotSide = robotSide;
      this.jointspaceTrajectoryCommand.set(jointspaceTrajectoryCommand);
      this.taskspaceTrajectoryCommand.set(taskspaceTrajectoryCommand);
   }

   public HandHybridJointspaceTaskspaceTrajectoryCommand(Random random)
   {
      this(RobotSide.generateRandomRobotSide(random), new SE3TrajectoryControllerCommand(random), new JointspaceTrajectoryCommand(random));
   }

   @Override
   public Class<HandHybridJointspaceTaskspaceTrajectoryMessage> getMessageClass()
   {
      return HandHybridJointspaceTaskspaceTrajectoryMessage.class;
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      robotSide = null;
      jointspaceTrajectoryCommand.clear();
      taskspaceTrajectoryCommand.clear();
   }

   @Override
   public void setFromMessage(HandHybridJointspaceTaskspaceTrajectoryMessage message)
   {
      FrameBasedCommand.super.setFromMessage(message);
   }

   @Override
   public void set(ReferenceFrameHashCodeResolver resolver, HandHybridJointspaceTaskspaceTrajectoryMessage message)
   {
      sequenceId = message.getSequenceId();
      robotSide = RobotSide.fromByte(message.getRobotSide());
      jointspaceTrajectoryCommand.setFromMessage(message.getJointspaceTrajectoryMessage());
      taskspaceTrajectoryCommand.set(resolver, message.getTaskspaceTrajectoryMessage());
   }

   @Override
   public boolean isCommandValid()
   {
      return robotSide != null && jointspaceTrajectoryCommand.isCommandValid() && taskspaceTrajectoryCommand.isCommandValid();
   }

   @Override
   public void set(HandHybridJointspaceTaskspaceTrajectoryCommand other)
   {
      sequenceId = other.sequenceId;
      robotSide = other.robotSide;
      taskspaceTrajectoryCommand.set(other.getTaskspaceTrajectoryCommand());
      jointspaceTrajectoryCommand.set(other.getJointspaceTrajectoryCommand());
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
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

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
