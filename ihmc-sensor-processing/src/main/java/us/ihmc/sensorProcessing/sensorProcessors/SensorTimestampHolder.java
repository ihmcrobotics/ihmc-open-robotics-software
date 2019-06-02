package us.ihmc.sensorProcessing.sensorProcessors;

public interface SensorTimestampHolder
{
   /**
    * @return machine timestamp in nanoseconds.
    */
   public long getTimestamp();

   /**
    * @return timestamp in nanoseconds representing the controller up-time.
    */
   public long getControllerTimestamp();

   // TODO Define me
   public long getSensorHeadPPSTimestamp();
}
