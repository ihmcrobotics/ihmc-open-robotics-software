package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.QueueableCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameBasedCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.hybridRigidBodyManager.ChestHybridJointspaceTaskspaceMessage;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class ChestHybridJointspaceTaskspaceTrajectoryCommand
      extends QueueableCommand<ChestHybridJointspaceTaskspaceTrajectoryCommand, ChestHybridJointspaceTaskspaceMessage> implements FrameBasedCommand<ChestHybridJointspaceTaskspaceMessage>
{
   private final SpineTrajectoryCommand jointspaceTrajectoryCommand = new SpineTrajectoryCommand();
   private final ChestTrajectoryCommand taskspaceTrajectoryCommand = new ChestTrajectoryCommand();

   public ChestHybridJointspaceTaskspaceTrajectoryCommand()
   {
   }

   public ChestHybridJointspaceTaskspaceTrajectoryCommand(ChestTrajectoryCommand taskspaceTrajectoryCommand, SpineTrajectoryCommand jointspaceTrajectoryCommand)
   {
      super();
      this.jointspaceTrajectoryCommand.set(jointspaceTrajectoryCommand);
      this.taskspaceTrajectoryCommand.set(taskspaceTrajectoryCommand);
   }

   @Override
   public void clear()
   {
      jointspaceTrajectoryCommand.clear();
      taskspaceTrajectoryCommand.clear();
   }

   @Override
   public void set(ChestHybridJointspaceTaskspaceMessage message)
   {
      jointspaceTrajectoryCommand.set(message.getSpineTrajectoryMessage());
      taskspaceTrajectoryCommand.set(message.getChestTrajectoryMessage());
   }
   
   @Override
   public void set(ReferenceFrame dataFrame, ReferenceFrame trajectoryFrame, ChestHybridJointspaceTaskspaceMessage message)
   {
      jointspaceTrajectoryCommand.set(message.getSpineTrajectoryMessage());
      taskspaceTrajectoryCommand.set(dataFrame, trajectoryFrame, message.getChestTrajectoryMessage());
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

   @Override
   public void addTimeOffset(double timeOffset)
   {
      taskspaceTrajectoryCommand.addTimeOffset(timeOffset);
      jointspaceTrajectoryCommand.addTimeOffset(timeOffset);
   }

   public SpineTrajectoryCommand getJointspaceTrajectoryCommand()
   {
      return jointspaceTrajectoryCommand;
   }

   public ChestTrajectoryCommand getTaskspaceTrajectoryCommand()
   {
      return taskspaceTrajectoryCommand;
   }

   @Override
   public Class<ChestHybridJointspaceTaskspaceMessage> getMessageClass()
   {
      return ChestHybridJointspaceTaskspaceMessage.class;
   }
}
