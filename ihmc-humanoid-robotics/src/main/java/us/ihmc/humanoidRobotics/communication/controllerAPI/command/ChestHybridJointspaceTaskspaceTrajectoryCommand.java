package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.Random;

import us.ihmc.communication.controllerAPI.command.QueueableCommand;
import us.ihmc.communication.packets.QueueableMessage;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameBasedCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.hybridRigidBodyManager.ChestHybridJointspaceTaskspaceTrajectoryMessage;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class ChestHybridJointspaceTaskspaceTrajectoryCommand
      extends QueueableCommand<ChestHybridJointspaceTaskspaceTrajectoryCommand, ChestHybridJointspaceTaskspaceTrajectoryMessage>
      implements FrameBasedCommand<ChestHybridJointspaceTaskspaceTrajectoryMessage>
{
   private final JointspaceTrajectoryCommand jointspaceTrajectoryCommand = new JointspaceTrajectoryCommand();
   private final ChestTrajectoryCommand taskspaceTrajectoryCommand = new ChestTrajectoryCommand();

   public ChestHybridJointspaceTaskspaceTrajectoryCommand()
   {
   }

   public ChestHybridJointspaceTaskspaceTrajectoryCommand(ChestTrajectoryCommand taskspaceTrajectoryCommand, JointspaceTrajectoryCommand jointspaceTrajectoryCommand)
   {
      super();
      this.jointspaceTrajectoryCommand.set(jointspaceTrajectoryCommand);
      this.taskspaceTrajectoryCommand.set(taskspaceTrajectoryCommand);
   }

   public ChestHybridJointspaceTaskspaceTrajectoryCommand(Random random)
   {
      this(new ChestTrajectoryCommand(random), new JointspaceTrajectoryCommand(random));
   }

   @Override
   public void clear()
   {
      jointspaceTrajectoryCommand.clear();
      taskspaceTrajectoryCommand.clear();
   }

   @Override
   public void set(ChestHybridJointspaceTaskspaceTrajectoryMessage message)
   {
      jointspaceTrajectoryCommand.set(message.getSpineTrajectoryMessage());
      taskspaceTrajectoryCommand.set(message.getChestTrajectoryMessage());
      setQueueableCommandVariables(message.getUniqueId(), message.getQueueingProperties());
   }

   @Override
   public void set(ReferenceFrameHashCodeResolver resolver, ChestHybridJointspaceTaskspaceTrajectoryMessage message)
   {
      jointspaceTrajectoryCommand.set(message.getSpineTrajectoryMessage());
      taskspaceTrajectoryCommand.set(resolver, message.getChestTrajectoryMessage());
      setQueueableCommandVariables(message.getUniqueId(), message.getQueueingProperties());
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

   public ChestTrajectoryCommand getTaskspaceTrajectoryCommand()
   {
      return taskspaceTrajectoryCommand;
   }

   @Override
   public Class<ChestHybridJointspaceTaskspaceTrajectoryMessage> getMessageClass()
   {
      return ChestHybridJointspaceTaskspaceTrajectoryMessage.class;
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
