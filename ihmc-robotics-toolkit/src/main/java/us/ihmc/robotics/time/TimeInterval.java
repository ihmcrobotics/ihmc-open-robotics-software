package us.ihmc.robotics.time;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tools.EuclidCoreIOTools;

public class TimeInterval
{
   private double startTime;
   private double endTime;

   public TimeInterval()
   {
      this(0.0, 0.0);
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

   public TimeInterval shiftInterval(double shiftTime)
   {
      this.startTime = getStartTime() + shiftTime;
      this.endTime = getEndTime() + shiftTime;
      return this;
   }

   public void get(TimeInterval timeIntervalToPack)
   {
      timeIntervalToPack.set(this);
   }

   public void set(TimeInterval timeInterval)
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
      return EuclidCoreIOTools.getStringOf("(", " )", ", ", startTime, endTime);
   }

   private void checkInterval()
   {
      if (endTime < startTime)
         throw new IllegalArgumentException("The end time is not valid! End time " + endTime + " must be greater than start time " + startTime);
   }
}
