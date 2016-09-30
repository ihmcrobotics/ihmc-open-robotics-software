package us.ihmc.quadrupedRobotics.util;

public class TimedValue implements TimeIntervalProvider
{
   private int value;
   private final TimeInterval timeInterval;

   public TimedValue()
   {
      this.value = 0;
      this.timeInterval = new TimeInterval();
   }

   public TimedValue(int value, TimeInterval timeInterval)
   {
      this.value = value;
      this.timeInterval = new TimeInterval(timeInterval);
   }

   public void set(TimedValue other)
   {
      this.value = other.value;
      this.timeInterval.set(other.timeInterval);
   }

   public int getValue()
   {
      return value;
   }

   @Override
   public TimeInterval getTimeInterval()
   {
      return timeInterval;
   }
}
