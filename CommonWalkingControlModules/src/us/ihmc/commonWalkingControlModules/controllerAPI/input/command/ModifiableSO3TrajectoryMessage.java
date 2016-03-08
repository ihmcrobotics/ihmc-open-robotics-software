package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import us.ihmc.humanoidRobotics.communication.packets.AbstractSO3TrajectoryMessage;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSO3TrajectoryPointList;

public abstract class ModifiableSO3TrajectoryMessage<T extends ModifiableSO3TrajectoryMessage<T, M>, M extends AbstractSO3TrajectoryMessage<M>>
      extends FrameSO3TrajectoryPointList implements ControllerMessage<T, M>
{
   public ModifiableSO3TrajectoryMessage()
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
