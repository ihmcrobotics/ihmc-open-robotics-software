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

   default void get(TimeIntervalBasics timeIntervalToPack)
   {
      timeIntervalToPack.set(this);
   }


   default boolean epsilonEquals(TimeIntervalReadOnly other, double epsilon)
   {
      return MathTools.epsilonEquals(getStartTime(), other.getStartTime(), epsilon) && MathTools.epsilonEquals(getEndTime(), other.getEndTime(), epsilon);
   }

   default boolean intervalContains(double time)
   {
      return MathTools.intervalContains(time, getStartTime(), getEndTime());
   }



   default void checkInterval()
   {
      if (getEndTime() < getStartTime())
         throw new IllegalArgumentException("The end time is not valid! End time " + getEndTime() + " must be greater than start time " + getStartTime());
   }
}
