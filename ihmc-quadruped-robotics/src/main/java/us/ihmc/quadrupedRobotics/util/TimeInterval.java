package us.ihmc.quadrupedRobotics.util;

import us.ihmc.robotics.MathTools;

public class TimeInterval
{
   private double startTime;
   private double endTime;

   public TimeInterval()
   {
      this.startTime = 0;
      this.endTime = 0;
   }

   public TimeInterval(TimeInterval timeInterval)
   {
      this.startTime = timeInterval.startTime;
      this.endTime = timeInterval.endTime;
   }

   public TimeInterval(double startTime, double endTime)
   {
      this.startTime = startTime;
      this.endTime = endTime;
   }

   public double getStartTime()
   {
      return startTime;
   }

   public void setStartTime(double startTime)
   {
      this.startTime = startTime;
   }

   public double getEndTime()
   {
      return endTime;
   }

   public void setEndTime(double endTime)
   {
      this.endTime = endTime;
   }

   public void setInterval(double startTime, double endTime)
   {
      setStartTime(startTime);
      setEndTime(endTime);
   }
   public double getDuration()
   {
      return getEndTime() - getStartTime();
   }

   public TimeInterval shiftInterval(double shiftTime)
   {
      setStartTime(getStartTime() + shiftTime);
      setEndTime(getEndTime() + shiftTime);
      return this;
   }

   public void get(TimeInterval timeInterval)
   {
      timeInterval.setStartTime(getStartTime());
      timeInterval.setEndTime(getEndTime());
   }

   public void set(TimeInterval timeInterval)
   {
      setStartTime(timeInterval.getStartTime());
      setEndTime(timeInterval.getEndTime());
   }

   public boolean epsilonEquals(TimeInterval other, double epsilon)
   {
      return MathTools.epsilonEquals(getStartTime(), other.getStartTime(), epsilon) && MathTools.epsilonEquals(getEndTime(), other.getEndTime(), epsilon);
   }
}
