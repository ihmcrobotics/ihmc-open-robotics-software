package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import us.ihmc.humanoidRobotics.communication.packets.AbstractSO3TrajectoryMessage;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSO3TrajectoryPointList;

public abstract class SO3TrajectoryControllerCommand<T extends SO3TrajectoryControllerCommand<T, M>, M extends AbstractSO3TrajectoryMessage<M>>
      extends FrameSO3TrajectoryPointList implements ControllerCommand<T, M>
{
   public SO3TrajectoryControllerCommand()
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
}
