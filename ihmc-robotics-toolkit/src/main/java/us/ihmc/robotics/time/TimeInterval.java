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
      this.startTime = startTime;
      this.endTime = endTime;

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

   public void setInterval(double startTime, double endTime)
   {
      this.startTime = startTime;
      this.endTime = endTime;
   }

   public double getDuration()
   {
      return getEndTime() - getStartTime();
   }


   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getStringOf("(", " )", ", ", getStartTime(), getEndTime());
   }
}
