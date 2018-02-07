package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.Random;

import us.ihmc.communication.controllerAPI.command.QueueableCommand;
import us.ihmc.communication.packets.QueueableMessage;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameBasedCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.hybridRigidBodyManager.HandHybridJointspaceTaskspaceTrajectoryMessage;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class HandHybridJointspaceTaskspaceTrajectoryCommand
      extends QueueableCommand<HandHybridJointspaceTaskspaceTrajectoryCommand, HandHybridJointspaceTaskspaceTrajectoryMessage>
      implements FrameBasedCommand<HandHybridJointspaceTaskspaceTrajectoryMessage>
{
   private RobotSide robotSide;
   private final JointspaceTrajectoryCommand jointspaceTrajectoryCommand = new JointspaceTrajectoryCommand();
   private final HandTrajectoryCommand taskspaceTrajectoryCommand = new HandTrajectoryCommand();

   public HandHybridJointspaceTaskspaceTrajectoryCommand()
   {
   }

   public HandHybridJointspaceTaskspaceTrajectoryCommand(HandTrajectoryCommand taskspaceTrajectoryCommand, JointspaceTrajectoryCommand jointspaceTrajectoryCommand)
   {
      robotSide = taskspaceTrajectoryCommand.getRobotSide();
      this.jointspaceTrajectoryCommand.set(jointspaceTrajectoryCommand);
      this.taskspaceTrajectoryCommand.set(taskspaceTrajectoryCommand);
   }

   public HandHybridJointspaceTaskspaceTrajectoryCommand(Random random)
   {
      this(new HandTrajectoryCommand(random), new JointspaceTrajectoryCommand(random));
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
      taskspaceTrajectoryCommand.set(message.getHandTrajectoryMessage());
      setQueueableCommandVariables(message.getUniqueId(), message.getQueueingProperties());
   }

   @Override
   public void set(ReferenceFrameHashCodeResolver resolver, HandHybridJointspaceTaskspaceTrajectoryMessage message)
   {
      robotSide = message.getRobotSide();
      jointspaceTrajectoryCommand.set(message.getJointspaceTrajectoryMessage());
      taskspaceTrajectoryCommand.set(resolver, message.getHandTrajectoryMessage());
      setQueueableCommandVariables(message.getUniqueId(), message.getQueueingProperties());
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
      setQueueableCommandVariables(other);
   }

   @Override
   public void addTimeOffset(double timeOffset)
   {
      taskspaceTrajectoryCommand.addTimeOffset(timeOffset);
      jointspaceTrajectoryCommand.addTimeOffset(timeOffset);
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public JointspaceTrajectoryCommand getJointspaceTrajectoryCommand()
   {
      return jointspaceTrajectoryCommand;
   }

   public HandTrajectoryCommand getTaskspaceTrajectoryCommand()
   {
      return taskspaceTrajectoryCommand;
   }

   @Override
   public void setQueueableCommandVariables(long messageId, QueueableMessage messageQueueingProperties)
   {
      // this override is needed to correctly store queuing information into the sub-messages
      super.setQueueableCommandVariables(messageId, messageQueueingProperties);
      jointspaceTrajectoryCommand.setQueueableCommandVariables(messageId, messageQueueingProperties);
      taskspaceTrajectoryCommand.setQueueableCommandVariables(messageId, messageQueueingProperties);
   }

   @Override
   public void setQueueableCommandVariables(QueueableCommand<?, ?> other)
   {
      // this override is needed to correctly store queuing information into the sub-messages
      taskspaceTrajectoryCommand.setQueueableCommandVariables(other);
      jointspaceTrajectoryCommand.setQueueableCommandVariables(other);
      super.setQueueableCommandVariables(other);
   }
}
