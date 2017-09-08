package us.ihmc.sensorProcessing.stateEstimation;

import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;

public interface SensorProcessingConfiguration
{
   public abstract void configureSensorProcessing(SensorProcessing sensorProcessing);

   public abstract SensorNoiseParameters getSensorNoiseParameters();
   
   public abstract double getEstimatorDT();
}
