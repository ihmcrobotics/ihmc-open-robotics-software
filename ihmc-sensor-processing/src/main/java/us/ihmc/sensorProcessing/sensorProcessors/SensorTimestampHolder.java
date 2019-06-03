package us.ihmc.sensorProcessing.sensorProcessors;

public interface SensorTimestampHolder
{
   /**
    * @return machine timestamp in nanoseconds.
    */
   public long getWallTime();

   /**
    * @return timestamp in nanoseconds representing the controller up-time.
    */
   public long getMonotonicTime();

   // TODO Define me
   public long getSyncTimestamp();
}
