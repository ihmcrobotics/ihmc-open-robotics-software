package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.humanoidRobotics.communication.packets.walking.SpineTrajectoryMessage;

public class SpineTrajectoryCommand extends JointspaceTrajectoryCommand<SpineTrajectoryCommand, SpineTrajectoryMessage>
{
   public SpineTrajectoryCommand()
   {
   }

   @Override
   public Class<SpineTrajectoryMessage> getMessageClass()
   {
      return SpineTrajectoryMessage.class;
   }
}
