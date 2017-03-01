package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.humanoidRobotics.communication.packets.walking.NeckTrajectoryMessage;

public class NeckTrajectoryCommand extends JointspaceTrajectoryCommand<NeckTrajectoryCommand, NeckTrajectoryMessage>
{
   public NeckTrajectoryCommand()
   {
      super();
   }

   @Override
   public Class<NeckTrajectoryMessage> getMessageClass()
   {
      return NeckTrajectoryMessage.class;
   }
}
