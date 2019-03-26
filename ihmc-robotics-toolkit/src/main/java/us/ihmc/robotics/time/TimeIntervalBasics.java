package us.ihmc.robotics.time;

import us.ihmc.commons.MathTools;

public interface TimeIntervalBasics extends TimeIntervalReadOnly
{
   /**
    * Use the interval setter when using this method! Otherwise, an incorrect interval may be set.
    */
   @Deprecated
   void setStartTime(double startTime);

   /**
    * Use the interval setter when using this method! Otherwise, an incorrect interval may be set.
    */
   @Deprecated
   void setEndTime(double endTime);

   default void reset()
   {
      setInterval(Double.NaN, Double.NaN);
   }

   void setInterval(double startTime, double endTime);

   default void set(TimeIntervalReadOnly timeInterval)
   {
      setInterval(timeInterval.getStartTime(), timeInterval.getEndTime());
      checkInterval();
   }

   default TimeIntervalBasics shiftInterval(double shiftTime)
   {
      setInterval(getStartTime() + shiftTime, getEndTime() + shiftTime);
      return this;
   }

   default boolean epsilonEquals(TimeInterval other, double epsilon)
   {
      return MathTools.epsilonEquals(getStartTime(), other.getStartTime(), epsilon) && MathTools.epsilonEquals(getEndTime(), other.getEndTime(), epsilon);
   }

   default boolean intervalContains(double time)
   {
      return MathTools.intervalContains(time, getStartTime(), getEndTime());
   }
}
