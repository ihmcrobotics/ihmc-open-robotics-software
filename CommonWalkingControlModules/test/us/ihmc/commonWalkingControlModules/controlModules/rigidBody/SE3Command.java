package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SE3TrajectoryControllerCommand;

public class SE3Command extends SE3TrajectoryControllerCommand<SE3Command, SE3Message>
{

   @Override
   public Class<SE3Message> getMessageClass()
   {
      return SE3Message.class;
   }

}
