package us.ihmc.sensorProcessing.stateEstimation;

import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;

public interface SensorProcessingConfiguration
{
   public abstract void configureSensorProcessing(SensorProcessing sensorProcessing);

   default SensorNoiseParameters getSensorNoiseParameters()
   {
      return null;
   }

   public abstract double getEstimatorDT();
}
