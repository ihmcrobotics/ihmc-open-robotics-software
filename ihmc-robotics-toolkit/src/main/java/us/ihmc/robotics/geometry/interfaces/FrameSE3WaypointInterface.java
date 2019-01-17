package us.ihmc.robotics.geometry.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public interface FrameSE3WaypointInterface<T extends FrameSE3WaypointInterface<T>> extends FrameEuclideanWaypointInterface<T>, FrameSO3WaypointInterface<T>
{
   @Override
   default boolean epsilonEquals(T other, double epsilon)
   {
      boolean euclideanMatch = FrameEuclideanWaypointInterface.super.epsilonEquals(other, epsilon);
      boolean so3Match = FrameSO3WaypointInterface.super.epsilonEquals(other, epsilon);
      return euclideanMatch && so3Match;
   }

   @Override
   default boolean geometricallyEquals(T other, double epsilon)
   {
      boolean euclideanMatch = FrameEuclideanWaypointInterface.super.geometricallyEquals(other, epsilon);
      boolean so3Match = FrameSO3WaypointInterface.super.geometricallyEquals(other, epsilon);
      return euclideanMatch && so3Match;
   }

   @Override
   default void set(T other)
   {
      FrameEuclideanWaypointInterface.super.set(other);
      FrameSO3WaypointInterface.super.set(other);
   }

   @Override
   default void setToNaN()
   {
      FrameEuclideanWaypointInterface.super.setToNaN();
      FrameSO3WaypointInterface.super.setToNaN();
   }

   @Override
   default void setToZero()
   {
      FrameEuclideanWaypointInterface.super.setToZero();
      FrameSO3WaypointInterface.super.setToZero();
   }

   @Override
   default boolean containsNaN()
   {
      return FrameEuclideanWaypointInterface.super.containsNaN() || FrameSO3WaypointInterface.super.containsNaN();
   }

   @Override
   default void setToNaN(ReferenceFrame referenceFrame)
   {
      FrameEuclideanWaypointInterface.super.setToNaN(referenceFrame);
      FrameSO3WaypointInterface.super.setToNaN(referenceFrame);
   }

   @Override
   default void setToZero(ReferenceFrame referenceFrame)
   {
      FrameEuclideanWaypointInterface.super.setToZero(referenceFrame);
      FrameSO3WaypointInterface.super.setToZero(referenceFrame);
   }
}
