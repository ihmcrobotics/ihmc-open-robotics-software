package us.ihmc.quadrupedRobotics.estimator.stateEstimator;

import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;

public interface QuadrupedSensorReaderWrapper extends SensorReader
{
   void setSensorReader(SensorReader sensorReader);
}
