package us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameChangeable;
import us.ihmc.euclid.transform.interfaces.Transform;

public interface FrameTrajectoryPointListInterface<T extends TrajectoryPointInterface & FrameChangeable>
      extends TrajectoryPointListInterface<T>, FrameChangeable
{
   default void clear(ReferenceFrame referenceFrame)
   {
      clear();
      setReferenceFrame(referenceFrame);
   }

   default void setIncludingFrame(FrameTrajectoryPointListInterface<T> trajectoryPoints)
   {
      clear(trajectoryPoints.getReferenceFrame());
      set(trajectoryPoints);
   }

   default void set(FrameTrajectoryPointListInterface<T> trajectoryPoints)
   {
      checkReferenceFrameMatch(trajectoryPoints);
      TrajectoryPointListInterface.super.set(trajectoryPoints);
   }

   @Override
   default void applyTransform(Transform transform)
   {
      for (int i = 0; i < getNumberOfTrajectoryPoints(); i++)
      {
         getTrajectoryPoint(i).applyTransform(transform);
      }
   }

   @Override
   default void applyInverseTransform(Transform transform)
   {
      for (int i = 0; i < getNumberOfTrajectoryPoints(); i++)
      {
         getTrajectoryPoint(i).applyInverseTransform(transform);
      }
   }
}
