package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.tools.EuclidCoreTools;

public interface OneDoFWaypointReadOnly
{
   double getPosition();

   double getVelocity();

   default void get(OneDoFWaypointBasics otherToPack)
   {
      otherToPack.set(this);
   }

   default boolean equals(OneDoFWaypointReadOnly other)
   {
      return EuclidCoreTools.equals(getPosition(), other.getPosition()) && EuclidCoreTools.equals(getVelocity(), other.getVelocity());
   }

   default boolean epsilonEquals(OneDoFWaypointReadOnly other, double epsilon)
   {
      return EuclidCoreTools.epsilonEquals(getPosition(), other.getPosition(), epsilon)
            && EuclidCoreTools.epsilonEquals(getVelocity(), other.getVelocity(), epsilon);
   }

   default boolean containsNaN()
   {
      return Double.isNaN(getPosition()) || Double.isNaN(getVelocity());
   }

}