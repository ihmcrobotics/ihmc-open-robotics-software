package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.humanoidRobotics.communication.packets.walking.HeadTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.NeckTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.hybridRigidBodyManager.HeadHybridJointspaceTaskspaceMessage;

public class HeadHybridJointspaceTaskspaceTrajectoryCommand extends
      HybridSO3JointspaceTaskspaceTrajectoryCommand<HeadHybridJointspaceTaskspaceTrajectoryCommand, HeadHybridJointspaceTaskspaceMessage, HeadTrajectoryCommand, HeadTrajectoryMessage, NeckTrajectoryCommand, NeckTrajectoryMessage>
{
   public HeadHybridJointspaceTaskspaceTrajectoryCommand()
   {
      super();
      jointspaceTrajectoryCommand = new NeckTrajectoryCommand();
      taskspaceTrajectoryCommand = new HeadTrajectoryCommand();
   }
   
   public HeadHybridJointspaceTaskspaceTrajectoryCommand(HeadTrajectoryCommand taskspaceTrajectoryCommand, NeckTrajectoryCommand jointspaceTrajectoryCommand)
   {
      super();
      jointspaceTrajectoryCommand = new NeckTrajectoryCommand();
      taskspaceTrajectoryCommand = new HeadTrajectoryCommand();
      jointspaceTrajectoryCommand.set(jointspaceTrajectoryCommand);
      taskspaceTrajectoryCommand.set(taskspaceTrajectoryCommand);
   }

   @Override
   public Class<HeadHybridJointspaceTaskspaceMessage> getMessageClass()
   {
      return HeadHybridJointspaceTaskspaceMessage.class;
   }
   
}
