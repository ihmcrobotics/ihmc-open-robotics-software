package us.ihmc.robotics.time;

import us.ihmc.commons.MathTools;
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

   public TimeIntervalBasics shiftInterval(double shiftTime)
   {
      this.startTime = getStartTime() + shiftTime;
      this.endTime = getEndTime() + shiftTime;
      return this;
   }

   public void set(TimeIntervalReadOnly timeInterval)
   {
      this.startTime = timeInterval.getStartTime();
      this.endTime = timeInterval.getEndTime();

      checkInterval();
   }

   public boolean epsilonEquals(TimeInterval other, double epsilon)
   {
      return MathTools.epsilonEquals(getStartTime(), other.getStartTime(), epsilon) && MathTools.epsilonEquals(getEndTime(), other.getEndTime(), epsilon);
   }

   public boolean intervalContains(double time)
   {
      return MathTools.intervalContains(time, getStartTime(), getEndTime());
   }

   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getStringOf("(", " )", ", ", getStartTime(), getEndTime());
   }
}
