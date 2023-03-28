package us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameChangeable;
import us.ihmc.euclid.transform.interfaces.Transform;

public interface FrameTrajectoryPointListBasics<T extends TrajectoryPointBasics & FrameChangeable> extends TrajectoryPointListBasics<T>, FrameChangeable
{
   default void clear(ReferenceFrame referenceFrame)
   {
      clear();
      setReferenceFrame(referenceFrame);
   }

   default void setIncludingFrame(FrameTrajectoryPointListBasics<T> trajectoryPoints)
   {
      clear(trajectoryPoints.getReferenceFrame());
      set(trajectoryPoints);
   }

   default void set(FrameTrajectoryPointListBasics<T> trajectoryPoints)
   {
      checkReferenceFrameMatch(trajectoryPoints);
      TrajectoryPointListBasics.super.set(trajectoryPoints);
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
