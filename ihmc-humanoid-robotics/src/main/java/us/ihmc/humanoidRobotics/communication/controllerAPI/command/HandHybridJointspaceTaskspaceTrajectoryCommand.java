package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.Random;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameBasedCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.hybridRigidBodyManager.HandHybridJointspaceTaskspaceTrajectoryMessage;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class HandHybridJointspaceTaskspaceTrajectoryCommand
      implements Command<HandHybridJointspaceTaskspaceTrajectoryCommand, HandHybridJointspaceTaskspaceTrajectoryMessage>,
      FrameBasedCommand<HandHybridJointspaceTaskspaceTrajectoryMessage>
{
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
      robotSide = null;
      jointspaceTrajectoryCommand.clear();
      taskspaceTrajectoryCommand.clear();
   }

   @Override
   public void set(HandHybridJointspaceTaskspaceTrajectoryMessage message)
   {
      robotSide = message.getRobotSide();
      jointspaceTrajectoryCommand.set(message.getJointspaceTrajectoryMessage());
      taskspaceTrajectoryCommand.set(message.getTaskspaceTrajectoryMessage());
   }

   @Override
   public void set(ReferenceFrameHashCodeResolver resolver, HandHybridJointspaceTaskspaceTrajectoryMessage message)
   {
      robotSide = message.getRobotSide();
      jointspaceTrajectoryCommand.set(message.getJointspaceTrajectoryMessage());
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
}
