package us.ihmc.robotics.time;

import us.ihmc.euclid.tools.EuclidCoreIOTools;

public class TimeInterval implements TimeIntervalBasics
{
   private double startTime;
   private double endTime;

   public TimeInterval()
   {
      this(Double.NaN, Double.NaN);
   }

   public TimeInterval(TimeInterval timeInterval)
   {
      this(timeInterval.getStartTime(), timeInterval.getEndTime());
   }

   public TimeInterval(double startTime, double endTime)
   {
      setInterval(startTime, endTime);
      checkInterval();
   }

   public double getStartTime()
   {
      return startTime;
   }

   /**
    * Use the interval setter when using this method! Otherwise, an incorrect interval may be set.
    */
   @Deprecated
   public void setStartTime(double startTime)
   {
      this.startTime = startTime;
   }

   public double getEndTime()
   {
      return endTime;
   }

   /**
    * Use the interval setter when using this method! Otherwise, an incorrect interval may be set.
    */
   @Deprecated
   public void setEndTime(double endTime)
   {
      this.endTime = endTime;
   }

   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getStringOf("(", " )", ", ", getStartTime(), getEndTime());
   }
}
