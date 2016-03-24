package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.packets.AbstractSE3TrajectoryMessage;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSE3TrajectoryPointList;

public abstract class SE3TrajectoryControllerCommand<T extends SE3TrajectoryControllerCommand<T, M>, M extends AbstractSE3TrajectoryMessage<M>>
      extends FrameSE3TrajectoryPointList implements Command<T, M>
{

   public SE3TrajectoryControllerCommand()
   {
   }

   @Override
   public void set(T other)
   {
      setIncludingFrame(other);
   }

   @Override
   public void set(M message)
   {
      message.getTrajectoryPoints(this);
   }

   @Override
   public boolean isCommandValid()
   {
      return getNumberOfTrajectoryPoints() > 0;
   }
}
