package us.ihmc.sensorProcessing.simulatedSensors;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.DoubleYoVariable;
import us.ihmc.sensorProcessing.stateEstimation.SensorProcessingConfiguration;

public class SimulatedSensorHolderAndReaderWithYawDriftCorruptor extends SimulatedSensorHolderAndReader
{
   public SimulatedSensorHolderAndReaderWithYawDriftCorruptor(StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions,
         SensorProcessingConfiguration sensorProcessingConfiguration, DoubleYoVariable yoTime, YoVariableRegistry parentRegistry)
   {
      super(stateEstimatorSensorDefinitions, sensorProcessingConfiguration, yoTime, parentRegistry);
   }
}
