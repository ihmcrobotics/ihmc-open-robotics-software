package us.ihmc.robotics.math.trajectories.waypoints.interfaces;

import us.ihmc.euclid.tools.EuclidCoreTools;

public interface OneDoFWaypointReadOnly
{
   double getPosition();

   double getVelocity();

   double getAcceleration();

   default void get(OneDoFWaypointBasics otherToPack)
   {
      otherToPack.set(this);
   }

   default boolean equals(OneDoFWaypointReadOnly other)
   {
      return EuclidCoreTools.equals(getPosition(), other.getPosition())
             && EuclidCoreTools.equals(getVelocity(), other.getVelocity())
             && EuclidCoreTools.equals(getAcceleration(), other.getAcceleration());
   }

   default boolean epsilonEquals(OneDoFWaypointReadOnly other, double epsilon)
   {
      return EuclidCoreTools.epsilonEquals(getPosition(), other.getPosition(), epsilon)
            && EuclidCoreTools.epsilonEquals(getVelocity(), other.getVelocity(), epsilon)
            && EuclidCoreTools.epsilonEquals(getAcceleration(), other.getAcceleration(), epsilon);
   }

   default boolean containsNaN()
   {
      return Double.isNaN(getPosition()) || Double.isNaN(getVelocity()) || Double.isNaN(getAcceleration());
   }

}