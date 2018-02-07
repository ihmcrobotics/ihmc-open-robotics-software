package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.Random;

import us.ihmc.communication.controllerAPI.command.QueueableCommand;
import us.ihmc.communication.packets.QueueableMessage;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameBasedCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.hybridRigidBodyManager.HeadHybridJointspaceTaskspaceTrajectoryMessage;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class HeadHybridJointspaceTaskspaceTrajectoryCommand
      extends QueueableCommand<HeadHybridJointspaceTaskspaceTrajectoryCommand, HeadHybridJointspaceTaskspaceTrajectoryMessage>
      implements FrameBasedCommand<HeadHybridJointspaceTaskspaceTrajectoryMessage>
{
   private final JointspaceTrajectoryCommand jointspaceTrajectoryCommand = new JointspaceTrajectoryCommand();
   private final HeadTrajectoryCommand taskspaceTrajectoryCommand = new HeadTrajectoryCommand();

   public HeadHybridJointspaceTaskspaceTrajectoryCommand()
   {
   }

   public HeadHybridJointspaceTaskspaceTrajectoryCommand(HeadTrajectoryCommand taskspaceTrajectoryCommand, JointspaceTrajectoryCommand jointspaceTrajectoryCommand)
   {
      this.jointspaceTrajectoryCommand.set(jointspaceTrajectoryCommand);
      this.taskspaceTrajectoryCommand.set(taskspaceTrajectoryCommand);
   }

   public HeadHybridJointspaceTaskspaceTrajectoryCommand(Random random)
   {
      this(new HeadTrajectoryCommand(random), new JointspaceTrajectoryCommand(random));
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
      taskspaceTrajectoryCommand.set(message.getHeadTrajectoryMessage());
      setQueueableCommandVariables(message.getUniqueId(), message.getQueueingProperties());
   }

   @Override
   public void set(ReferenceFrameHashCodeResolver resolver, HeadHybridJointspaceTaskspaceTrajectoryMessage message)
   {
      jointspaceTrajectoryCommand.set(message.getJointspaceTrajectoryMessage());
      taskspaceTrajectoryCommand.set(resolver, message.getHeadTrajectoryMessage());
      setQueueableCommandVariables(message.getUniqueId(), message.getQueueingProperties());
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
      setQueueableCommandVariables(other);
   }

   @Override
   public void addTimeOffset(double timeOffset)
   {
      taskspaceTrajectoryCommand.addTimeOffset(timeOffset);
      jointspaceTrajectoryCommand.addTimeOffset(timeOffset);
   }

   public JointspaceTrajectoryCommand getJointspaceTrajectoryCommand()
   {
      return jointspaceTrajectoryCommand;
   }

   public HeadTrajectoryCommand getTaskspaceTrajectoryCommand()
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
