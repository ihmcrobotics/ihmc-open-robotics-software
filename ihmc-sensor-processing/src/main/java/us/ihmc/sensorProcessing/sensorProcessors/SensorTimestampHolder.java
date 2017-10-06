package us.ihmc.sensorProcessing.sensorProcessors;

public interface SensorTimestampHolder
{
   public long getTimestamp();

   public long getVisionSensorTimestamp();

   public long getSensorHeadPPSTimestamp();
}
