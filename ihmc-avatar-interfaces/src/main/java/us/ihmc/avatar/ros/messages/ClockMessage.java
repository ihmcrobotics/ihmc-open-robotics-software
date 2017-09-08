package us.ihmc.avatar.ros.messages;

import java.nio.LongBuffer;

public class ClockMessage
{
   /**
    * General Clock message wrapper, as defined in the rosgrap_msgs/Clock rosmsg type.
    *
    * Clock messages hang on to seconds and nanoseconds as ints.  Intermediate representations
    * are obtained by performing artithmetic on the nanoseconds portion.
    */

   private long secs;
   private long nsecs;

   public void setFromBuffer(LongBuffer buffer)
   {
      buffer.rewind();
      secs = buffer.get();
      nsecs = buffer.get();
   }

   public void packTime(long[] time)
   {
      if(time.length < 2)
         throw new RuntimeException("Array for packing time not large enough");

      time[0] = secs;
      time[1] = nsecs;
   }

   public long secs()
   {
      return secs;
   }

   public long nsecs()
   {
      return nsecs;
   }

   public long toNSecs()
   {
      return (secs * 1000000000) + nsecs;
   }

   public void setSecs(long secs)
   {
      this.secs = secs;
   }

   public void setNSecs(long nsecs)
   {
      this.nsecs = nsecs;
   }

   @Override
   public String toString()
   {
      StringBuilder builder = new StringBuilder();
      builder.append("[TimeMessage] Secs: " + secs);
      builder.append("[TimeMessage} NSecs: " + nsecs);

      return builder.toString();
   }
}
