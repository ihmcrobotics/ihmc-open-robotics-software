package us.ihmc.sensorProcessing.sensorProcessors;

public interface SensorTimestampHolder
{
   /**
    * Gets the time in nanoseconds of the clock hanging on the wall. Takes into account leap
    * seconds/years and is updated by the NTP server (thus can jump backwards). The wall time is
    * usually used in ROS1 for synchronizing timestamps of different time sources (computers,
    * sensors, etc.).
    * 
    * @return wall-time in nanoseconds.
    */
   public long getWallTime();

   /**
    * Gets the time in nanoseconds that represents the absolute elapsed wall-clock time since some
    * arbitrary, fixed point in the past. It is not affected by changes in the system time-of-day
    * clock. This time is usually computed from a real-time process and can be used for reliably
    * computing the time elapsed between two events.
    * 
    * @return monotonic time in nanoseconds.
    */
   public long getMonotonicTime();

   /**
    * Platform dependent.
    * <p>
    * Time signal in nanoseconds that can be used to synchronize two time sources.
    * </p>
    * 
    * @return the timestamp for synchronization.
    */
   public long getSyncTimestamp();
}
