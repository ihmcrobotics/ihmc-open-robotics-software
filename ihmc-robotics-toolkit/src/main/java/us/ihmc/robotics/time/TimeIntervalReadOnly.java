package us.ihmc.robotics.time;

import us.ihmc.commons.MathTools;

public interface TimeIntervalReadOnly
{
   double getStartTime();

   double getEndTime();

   default double getDuration()
   {
      return getEndTime() - getStartTime();
   }

   default boolean epsilonEquals(TimeIntervalReadOnly other, double epsilon)
   {
      return MathTools.epsilonEquals(getStartTime(), other.getStartTime(), epsilon) && MathTools.epsilonEquals(getEndTime(), other.getEndTime(), epsilon);
   }

   default boolean intervalContains(double time)
   {
      return MathTools.intervalContains(time, getStartTime(), getEndTime());
   }

   default boolean epsilonContains(double time, double epsilon)
   {
      return MathTools.intervalContains(time, getStartTime() - epsilon, getEndTime() + epsilon);
   }

   default void checkInterval()
   {
      checkInterval(this);
   }

   static void checkInterval(TimeIntervalReadOnly timeInterval)
   {
      if (timeInterval.getEndTime() < timeInterval.getStartTime())
         throw new IllegalArgumentException(
               "The end time is not valid! End time " + timeInterval.getEndTime() + " must be greater than start time " + timeInterval.getStartTime());
   }
}
