package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.QueueableCommand;
import us.ihmc.communication.packets.QueueableMessage;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameBasedCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.hybridRigidBodyManager.HandHybridJointspaceTaskspaceTrajectoryMessage;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

import java.util.Random;

public class HandHybridJointspaceTaskspaceTrajectoryCommand
      extends QueueableCommand<HandHybridJointspaceTaskspaceTrajectoryCommand, HandHybridJointspaceTaskspaceTrajectoryMessage>
      implements FrameBasedCommand<HandHybridJointspaceTaskspaceTrajectoryMessage>
{
   private final ArmTrajectoryCommand jointspaceTrajectoryCommand = new ArmTrajectoryCommand();
   private final HandTrajectoryCommand taskspaceTrajectoryCommand = new HandTrajectoryCommand();

   public HandHybridJointspaceTaskspaceTrajectoryCommand()
   {
   }

   public HandHybridJointspaceTaskspaceTrajectoryCommand(HandTrajectoryCommand taskspaceTrajectoryCommand, ArmTrajectoryCommand jointspaceTrajectoryCommand)
   {
      super();
      this.jointspaceTrajectoryCommand.set(jointspaceTrajectoryCommand);
      this.taskspaceTrajectoryCommand.set(taskspaceTrajectoryCommand);
   }

   public HandHybridJointspaceTaskspaceTrajectoryCommand(Random random)
   {
      this(new HandTrajectoryCommand(random), new ArmTrajectoryCommand(random));
   }

   @Override
   public Class<HandHybridJointspaceTaskspaceTrajectoryMessage> getMessageClass()
   {
      return HandHybridJointspaceTaskspaceTrajectoryMessage.class;
   }

   @Override
   public void clear()
   {
      jointspaceTrajectoryCommand.clear();
      taskspaceTrajectoryCommand.clear();
   }

   @Override
   public void set(HandHybridJointspaceTaskspaceTrajectoryMessage message)
   {
      jointspaceTrajectoryCommand.set(message.getArmTrajectoryMessage());
      taskspaceTrajectoryCommand.set(message.getHandTrajectoryMessage());
      setQueueableCommandVariables(message);
   }

   @Override
   public void set(ReferenceFrameHashCodeResolver resolver, HandHybridJointspaceTaskspaceTrajectoryMessage message)
   {
      jointspaceTrajectoryCommand.set(message.getArmTrajectoryMessage());
      taskspaceTrajectoryCommand.set(resolver, message.getHandTrajectoryMessage());
      setQueueableCommandVariables(message);
   }

   @Override
   public boolean isCommandValid()
   {
      return jointspaceTrajectoryCommand.isCommandValid() && taskspaceTrajectoryCommand.isCommandValid();
   }

   @Override
   public void set(HandHybridJointspaceTaskspaceTrajectoryCommand other)
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

   public ArmTrajectoryCommand getJointspaceTrajectoryCommand()
   {
      return jointspaceTrajectoryCommand;
   }

   public HandTrajectoryCommand getTaskspaceTrajectoryCommand()
   {
      return taskspaceTrajectoryCommand;
   }

   @Override
   public void setQueueableCommandVariables(QueueableMessage<?> message)
   {
      // this override is needed to correctly store queuing information into the sub-messages
      super.setQueueableCommandVariables(message);
      jointspaceTrajectoryCommand.setQueueableCommandVariables(message);
      taskspaceTrajectoryCommand.setQueueableCommandVariables(message);
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
