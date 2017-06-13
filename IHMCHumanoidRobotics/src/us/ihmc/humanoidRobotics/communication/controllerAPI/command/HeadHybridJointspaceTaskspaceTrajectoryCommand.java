package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.QueueableCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameBasedCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.hybridRigidBodyManager.HeadHybridJointspaceTaskspaceTrajectoryMessage;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class HeadHybridJointspaceTaskspaceTrajectoryCommand
      extends QueueableCommand<HeadHybridJointspaceTaskspaceTrajectoryCommand, HeadHybridJointspaceTaskspaceTrajectoryMessage>
      implements FrameBasedCommand<HeadHybridJointspaceTaskspaceTrajectoryMessage>
{
   private final NeckTrajectoryCommand jointspaceTrajectoryCommand = new NeckTrajectoryCommand();
   private final HeadTrajectoryCommand taskspaceTrajectoryCommand = new HeadTrajectoryCommand();

   public HeadHybridJointspaceTaskspaceTrajectoryCommand()
   {
   }

   public HeadHybridJointspaceTaskspaceTrajectoryCommand(HeadTrajectoryCommand taskspaceTrajectoryCommand, NeckTrajectoryCommand jointspaceTrajectoryCommand)
   {
      super();
      this.jointspaceTrajectoryCommand.set(jointspaceTrajectoryCommand);
      this.taskspaceTrajectoryCommand.set(taskspaceTrajectoryCommand);
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
      jointspaceTrajectoryCommand.set(message.getNeckTrajectoryMessage());
      taskspaceTrajectoryCommand.set(message.getHeadTrajectoryMessage());
      setQueueqableCommandVariables(message);
   }

   @Override
   public void set(ReferenceFrameHashCodeResolver resolver, HeadHybridJointspaceTaskspaceTrajectoryMessage message)
   {
      jointspaceTrajectoryCommand.set(message.getNeckTrajectoryMessage());
      taskspaceTrajectoryCommand.set(resolver, message.getHeadTrajectoryMessage());
      setQueueqableCommandVariables(message);
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
      setQueueqableCommandVariables(other);
   }

   @Override
   public void addTimeOffset(double timeOffset)
   {
      taskspaceTrajectoryCommand.addTimeOffset(timeOffset);
      jointspaceTrajectoryCommand.addTimeOffset(timeOffset);
   }

   public NeckTrajectoryCommand getJointspaceTrajectoryCommand()
   {
      return jointspaceTrajectoryCommand;
   }

   public HeadTrajectoryCommand getTaskspaceTrajectoryCommand()
   {
      return taskspaceTrajectoryCommand;
   }
}
