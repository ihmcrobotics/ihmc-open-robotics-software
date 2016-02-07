package us.ihmc.aware.util;

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

   public void get(TimeInterval timeInterval)
   {
      timeInterval.startTime = this.startTime;
      timeInterval.endTime = this.endTime;
   }

   public void set(TimeInterval timeInterval)
   {
      this.startTime = timeInterval.startTime;
      this.endTime = timeInterval.endTime;
   }

   public void setInterval(double startTime, double endTime)
   {
      this.startTime = startTime;
      this.endTime = endTime;
   }

   public double getDuration()
   {
      return endTime - startTime;
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

   public TimeInterval shiftInterval(double shiftTime)
   {
      this.startTime = this.startTime + shiftTime;
      this.endTime = this.endTime + shiftTime;
      return this;
   }
}
