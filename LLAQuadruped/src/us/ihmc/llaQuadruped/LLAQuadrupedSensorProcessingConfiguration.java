package us.ihmc.llaQuadruped;

import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.stateEstimation.SensorProcessingConfiguration;

public class LLAQuadrupedSensorProcessingConfiguration implements SensorProcessingConfiguration
{
   @Override
   public void configureSensorProcessing(SensorProcessing sensorProcessing)
   {
   }

   @Override
   public SensorNoiseParameters getSensorNoiseParameters()
   {
      return null;
   }

   @Override
   public double getEstimatorDT()
   {
      return 0.001;
   }
}
