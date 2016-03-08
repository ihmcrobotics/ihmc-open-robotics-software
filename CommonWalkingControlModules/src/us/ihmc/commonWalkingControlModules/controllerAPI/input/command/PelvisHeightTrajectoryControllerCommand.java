package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1DList;

public class PelvisHeightTrajectoryControllerCommand extends SimpleTrajectoryPoint1DList implements ControllerCommand<PelvisHeightTrajectoryControllerCommand, PelvisHeightTrajectoryMessage>
{
   public PelvisHeightTrajectoryControllerCommand()
   {
   }

   @Override
   public void set(PelvisHeightTrajectoryControllerCommand other)
   {
      super.set(other);
   }

   @Override
   public void set(PelvisHeightTrajectoryMessage message)
   {
      message.getTrajectoryPoints(this);
   }

   @Override
   public Class<PelvisHeightTrajectoryMessage> getMessageClass()
   {
      return PelvisHeightTrajectoryMessage.class;
   }
}
