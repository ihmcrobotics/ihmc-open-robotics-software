package us.ihmc.sensorProcessing.diagnostic;

import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;

public class DiagnosticSensorNoiseParameters extends SensorNoiseParameters
{
   public DiagnosticSensorNoiseParameters()
   {
      setOrientationMeasurementLatency(0.02);
      setAngularVelocityMeasurementLatency(0.02);

      setOrientationMeasurementStandardDeviation(0.001);
      setAngularVelocityMeasurementStandardDeviation(0.1);
      setLinearAccelerationMeasurementStandardDeviation(0.4);
   }
}
