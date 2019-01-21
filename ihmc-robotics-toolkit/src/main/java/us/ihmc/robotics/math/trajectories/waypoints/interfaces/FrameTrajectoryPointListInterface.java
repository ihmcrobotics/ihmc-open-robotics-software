package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameChangeable;
import us.ihmc.euclid.transform.interfaces.Transform;

public interface FrameTrajectoryPointListInterface<T extends TrajectoryPointInterface & FrameChangeable>
      extends TrajectoryPointListInterface<T>, FrameChangeable
{
   public default void clear(ReferenceFrame referenceFrame)
   {
      clear();
      setReferenceFrame(referenceFrame);
   }

   public default void setIncludingFrame(FrameTrajectoryPointListInterface<T> trajectoryPoints)
   {
      clear(trajectoryPoints.getReferenceFrame());
      set(trajectoryPoints);
   }

   @Override
   public default void applyTransform(Transform transform)
   {
      for (int i = 0; i < getNumberOfTrajectoryPoints(); i++)
      {
         getTrajectoryPoint(i).applyTransform(transform);
      }
   }

   @Override
   public default void applyInverseTransform(Transform transform)
   {
      for (int i = 0; i < getNumberOfTrajectoryPoints(); i++)
      {
         getTrajectoryPoint(i).applyInverseTransform(transform);
      }
   }
}
