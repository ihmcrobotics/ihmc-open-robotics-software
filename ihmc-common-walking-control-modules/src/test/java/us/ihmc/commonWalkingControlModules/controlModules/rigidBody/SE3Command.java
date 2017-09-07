package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SE3TrajectoryControllerCommand;

public class SE3Command extends SE3TrajectoryControllerCommand<SE3Command, SE3Message>
{
   public SE3Command()
   {
      super(ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
   }

   @Override
   public Class<SE3Message> getMessageClass()
   {
      return SE3Message.class;
   }

}
