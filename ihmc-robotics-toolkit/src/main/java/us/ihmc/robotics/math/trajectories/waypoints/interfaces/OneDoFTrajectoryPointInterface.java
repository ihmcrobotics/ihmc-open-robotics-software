package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.robotics.geometry.interfaces.OneDoFWaypointInterface;

public interface OneDoFTrajectoryPointInterface extends TrajectoryPointInterface, OneDoFWaypointInterface
{
   public default void set(double time, double position, double velocity)
   {
      setTime(time);
      set(position, velocity);
   }

   default void set(OneDoFTrajectoryPointInterface other)
   {
      setTime(other.getTime());
      OneDoFWaypointInterface.super.set(other);
   }

   default boolean epsilonEquals(OneDoFTrajectoryPointInterface other, double epsilon)
   {
      return EuclidCoreTools.epsilonEquals(getTime(), other.getTime(), epsilon) && OneDoFWaypointInterface.super.epsilonEquals(other, epsilon);
   }

   @Override
   default void setToZero()
   {
      setTimeToZero();
      OneDoFWaypointInterface.super.setToZero();
   }

   @Override
   default void setToNaN()
   {
      setTimeToNaN();
      OneDoFWaypointInterface.super.setToNaN();
   }

   @Override
   default boolean containsNaN()
   {
      return Double.isNaN(getTime()) || OneDoFWaypointInterface.super.containsNaN();
   }
}
