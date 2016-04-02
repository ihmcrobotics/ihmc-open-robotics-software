package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1DList;

public class PelvisHeightTrajectoryCommand extends SimpleTrajectoryPoint1DList implements Command<PelvisHeightTrajectoryCommand, PelvisHeightTrajectoryMessage>
{
   public PelvisHeightTrajectoryCommand()
   {
   }

   @Override
   public void set(PelvisHeightTrajectoryCommand other)
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

   @Override
   public boolean isCommandValid()
   {
      return getNumberOfTrajectoryPoints() > 0;
   }
}
