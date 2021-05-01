package us.ihmc.robotics.math.trajectories.interfaces;

import us.ihmc.robotics.time.TimeIntervalProvider;

public interface PolynomialReadOnly extends TimeIntervalProvider, DoubleTrajectoryGenerator
{
   int getMaximumNumberOfCoefficients();

   int getNumberOfCoefficients();

   double getCurrentTime();

   double getCoefficient(int idx);

   double[] getCoefficients();

   default double getFinalTime()
   {
      return getTimeInterval().getEndTime();
   }

   default double getInitialTime()
   {
      return getTimeInterval().getStartTime();
   }

   default double getDuration()
   {
      return getTimeInterval().getDuration();
   }

   default boolean timeIntervalContains(double timeToCheck, double EPSILON)
   {
      return getTimeInterval().epsilonContains(timeToCheck, EPSILON);
   }

   default boolean timeIntervalContains(double timeToCheck)
   {
      return getTimeInterval().intervalContains(timeToCheck);
   }

   @Override
   default boolean isDone()
   {
      return getCurrentTime() >= getTimeInterval().getEndTime();
   }
}
