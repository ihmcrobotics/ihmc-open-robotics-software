package us.ihmc.darpaRoboticsChallenge.ros.messages;

import java.nio.IntBuffer;

public class ClockMessage
{
   /**
    * General Clock message wrapper, as defined in the rosgrap_msgs/Clock rosmsg type.
    *
    * Clock messages hang on to seconds and nanoseconds as ints.  Intermediate representations
    * are obtained by performing artithmetic on the nanoseconds portion.
    */

   private int secs;
   private int nsecs;

   public void setFromBuffer(IntBuffer buffer)
   {
      buffer.rewind();
      secs = buffer.get();
      nsecs = buffer.get();
   }

   public void packTime(int[] time)
   {
      if(time.length < 2)
         throw new RuntimeException("Array for packing time not large enough");

      time[0] = secs;
      time[1] = nsecs;
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
