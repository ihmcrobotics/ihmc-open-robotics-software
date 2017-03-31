package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.SpineTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.hybridRigidBodyManager.ChestHybridJointspaceTaskspaceMessage;

public class ChestHybridJointspaceTaskspaceTrajectoryCommand extends
      HybridSO3JointspaceTaskspaceTrajectoryCommand<ChestHybridJointspaceTaskspaceTrajectoryCommand, ChestHybridJointspaceTaskspaceMessage, ChestTrajectoryCommand, ChestTrajectoryMessage, SpineTrajectoryCommand, SpineTrajectoryMessage>
{
   public ChestHybridJointspaceTaskspaceTrajectoryCommand()
   {
      super();
      jointspaceTrajectoryCommand = new SpineTrajectoryCommand();
      taskspaceTrajectoryCommand = new ChestTrajectoryCommand();
   }
   
   public ChestHybridJointspaceTaskspaceTrajectoryCommand(ChestTrajectoryCommand taskspaceTrajectoryCommand, SpineTrajectoryCommand jointspaceTrajectoryCommand)
   {
      super();
      jointspaceTrajectoryCommand = new SpineTrajectoryCommand();
      taskspaceTrajectoryCommand = new ChestTrajectoryCommand();
      jointspaceTrajectoryCommand.set(jointspaceTrajectoryCommand);
      taskspaceTrajectoryCommand.set(taskspaceTrajectoryCommand);
   }

   @Override
   public Class<ChestHybridJointspaceTaskspaceMessage> getMessageClass()
   {
      return ChestHybridJointspaceTaskspaceMessage.class;
   }
}
