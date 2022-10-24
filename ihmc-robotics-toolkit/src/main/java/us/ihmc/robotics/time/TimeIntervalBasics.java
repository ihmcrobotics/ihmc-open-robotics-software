package us.ihmc.robotics.time;

import us.ihmc.euclid.tools.EuclidCoreIOTools;

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
      setIntervalUnsafe(Double.NaN, Double.NaN);
   }

   default void setInterval(double startTime, double endTime)
   {
      setIntervalUnsafe(startTime, endTime);
      checkInterval();
   }

   default void setIntervalUnsafe(double startTime, double endTime)
   {
      setStartTime(startTime);
      setEndTime(endTime);
   }

   default void set(TimeIntervalReadOnly timeInterval)
   {
      setIntervalUnsafe(timeInterval.getStartTime(), timeInterval.getEndTime());
   }

   default TimeIntervalBasics shiftInterval(double shiftTime)
   {
      setIntervalUnsafe(getStartTime() + shiftTime, getEndTime() + shiftTime);
      return this;
   }

}
