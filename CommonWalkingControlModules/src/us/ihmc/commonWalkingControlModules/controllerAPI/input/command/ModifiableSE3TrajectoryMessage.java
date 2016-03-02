package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import us.ihmc.humanoidRobotics.communication.packets.AbstractSE3TrajectoryMessage;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSE3TrajectoryPointList;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public abstract class ModifiableSE3TrajectoryMessage<T extends ModifiableSE3TrajectoryMessage<T, M>, M extends AbstractSE3TrajectoryMessage<M>>
      extends FrameSE3TrajectoryPointList implements ControllerMessage<T, M>
{

   public ModifiableSE3TrajectoryMessage()
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
      setIncludingFrame(ReferenceFrame.getWorldFrame(), message);
   }
}
