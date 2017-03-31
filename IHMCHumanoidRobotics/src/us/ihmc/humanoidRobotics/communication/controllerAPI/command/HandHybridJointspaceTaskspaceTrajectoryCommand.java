package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.hybridRigidBodyManager.HandHybridJointspaceTaskspaceTrajectoryMessage;

public class HandHybridJointspaceTaskspaceTrajectoryCommand extends
      HybridSE3JointspaceTaskspaceTrajectoryCommand<HandHybridJointspaceTaskspaceTrajectoryCommand, HandHybridJointspaceTaskspaceTrajectoryMessage, HandTrajectoryCommand, HandTrajectoryMessage, ArmTrajectoryCommand, ArmTrajectoryMessage>
{
   public HandHybridJointspaceTaskspaceTrajectoryCommand()
   {
      super();
      jointspaceTrajectoryCommand = new ArmTrajectoryCommand();
      taskspaceTrajectoryCommand = new HandTrajectoryCommand();
   }
   
   public HandHybridJointspaceTaskspaceTrajectoryCommand(ArmTrajectoryCommand armTrajectoryCommand, HandTrajectoryCommand handTrajectoryCommand)
   {
      super();
      jointspaceTrajectoryCommand = new ArmTrajectoryCommand();
      taskspaceTrajectoryCommand = new HandTrajectoryCommand();
      jointspaceTrajectoryCommand.set(armTrajectoryCommand);
      taskspaceTrajectoryCommand.set(handTrajectoryCommand);
   }
   
   @Override
   public Class<HandHybridJointspaceTaskspaceTrajectoryMessage> getMessageClass()
   {
      return HandHybridJointspaceTaskspaceTrajectoryMessage.class;
   }
}
