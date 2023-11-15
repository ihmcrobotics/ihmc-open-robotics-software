package us.ihmc.sensorProcessing.outputData;

public interface LowLevelStateReadOnly
{
   double getPosition();
   double getVelocity();
   double getAcceleration();
   double getEffort();
   double getTemperature();
   boolean isPositionValid();
   boolean isVelocityValid();
   boolean isAccelerationValid();
   boolean isEffortValid();
   boolean isTemperatureValid();
}
