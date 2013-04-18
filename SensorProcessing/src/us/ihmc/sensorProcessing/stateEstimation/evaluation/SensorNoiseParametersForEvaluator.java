package us.ihmc.sensorProcessing.stateEstimation.evaluation;

import javax.vecmath.Vector3d;

import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;

public class SensorNoiseParametersForEvaluator
{
   public static SensorNoiseParameters createSensorNoiseParameters()
   {
      SensorNoiseParameters sensorNoiseParameters = new SensorNoiseParameters();

      sensorNoiseParameters.setComAccelerationProcessNoiseStandardDeviation(Math.sqrt(1e-1));
      sensorNoiseParameters.setAngularAccelerationProcessNoiseStandardDeviation(1e-1);
      
      sensorNoiseParameters.setOrientationMeasurementStandardDeviation(Math.sqrt(1e-2));
      sensorNoiseParameters.setAngularVelocityMeasurementStandardDeviation(1e-1);
      sensorNoiseParameters.setLinearAccelerationMeasurementStandardDeviation(1e0);
      sensorNoiseParameters.setPointVelocityMeasurementStandardDeviation(1e-1);

      sensorNoiseParameters.setAngularVelocityBiasProcessNoiseStandardDeviation(Math.sqrt(1e-5));
      sensorNoiseParameters.setLinearAccelerationBiasProcessNoiseStandardDeviation(Math.sqrt(1e-4));

      sensorNoiseParameters.setInitialLinearVelocityBias(new Vector3d());
      sensorNoiseParameters.setInitialAngularVelocityBias(new Vector3d());

      return sensorNoiseParameters;
   }

}
