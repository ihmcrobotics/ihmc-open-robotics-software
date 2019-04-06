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
      setInterval(Double.NaN, Double.NaN);
   }

   default void setInterval(double startTime, double endTime)
   {
      setStartTime(startTime);
      setEndTime(endTime);
      checkInterval();
   }

   default void set(TimeIntervalReadOnly timeInterval)
   {
      setInterval(timeInterval.getStartTime(), timeInterval.getEndTime());
   }

   default TimeIntervalBasics shiftInterval(double shiftTime)
   {
      setInterval(getStartTime() + shiftTime, getEndTime() + shiftTime);
      return this;
   }

}
