package us.ihmc.sensorProcessing.simulatedSensors;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.sensorProcessing.stateEstimation.SensorProcessingConfiguration;

public class SimulatedSensorHolderAndReaderWithYawDriftCorruptor extends SimulatedSensorHolderAndReader
{
   public SimulatedSensorHolderAndReaderWithYawDriftCorruptor(StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions,
         SensorProcessingConfiguration sensorProcessingConfiguration, YoDouble yoTime, YoRegistry parentRegistry)
   {
      super(stateEstimatorSensorDefinitions, sensorProcessingConfiguration, yoTime, parentRegistry);
   }
}
