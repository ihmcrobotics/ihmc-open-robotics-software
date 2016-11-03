package us.ihmc.sensorProcessing.stateEstimation.evaluation;

import javax.vecmath.Vector3d;

import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;

public class SensorNoiseParametersForEvaluator
{
   public static SensorNoiseParameters createSensorNoiseParameters()
   {
      SensorNoiseParameters sensorNoiseParameters = new SensorNoiseParameters();

      sensorNoiseParameters.setComAccelerationProcessNoiseStandardDeviation(0.3);
      sensorNoiseParameters.setAngularAccelerationProcessNoiseStandardDeviation(0.1);

      sensorNoiseParameters.setOrientationMeasurementStandardDeviation(Math.sqrt(0.01));
      sensorNoiseParameters.setAngularVelocityMeasurementStandardDeviation(0.1);
      sensorNoiseParameters.setLinearAccelerationMeasurementStandardDeviation(1.0);

      sensorNoiseParameters.setAngularVelocityBiasProcessNoiseStandardDeviation(0.003);
      sensorNoiseParameters.setLinearAccelerationBiasProcessNoiseStandardDeviation(0.01);

      sensorNoiseParameters.setInitialLinearVelocityBias(new Vector3d());
      sensorNoiseParameters.setInitialAngularVelocityBias(new Vector3d());

      return sensorNoiseParameters;
   }

   public static SensorNoiseParameters createLotsOfSensorNoiseParameters()
   {
      SensorNoiseParameters sensorNoiseParameters = new SensorNoiseParameters();

      sensorNoiseParameters.setComAccelerationProcessNoiseStandardDeviation(0.01);
      sensorNoiseParameters.setAngularAccelerationProcessNoiseStandardDeviation(0.001);

      sensorNoiseParameters.setOrientationMeasurementStandardDeviation(0.2);
      sensorNoiseParameters.setAngularVelocityMeasurementStandardDeviation(0.2);
      sensorNoiseParameters.setLinearAccelerationMeasurementStandardDeviation(2.0);

      sensorNoiseParameters.setAngularVelocityBiasProcessNoiseStandardDeviation(1e-6);
      sensorNoiseParameters.setLinearAccelerationBiasProcessNoiseStandardDeviation(1e-6);

      sensorNoiseParameters.setInitialLinearVelocityBias(new Vector3d());
      sensorNoiseParameters.setInitialAngularVelocityBias(new Vector3d());

      return sensorNoiseParameters;
   }

   public static SensorNoiseParameters createVeryLittleSensorNoiseParameters()
   {
      SensorNoiseParameters sensorNoiseParameters = new SensorNoiseParameters();

      sensorNoiseParameters.setComAccelerationProcessNoiseStandardDeviation(1e-4);
      sensorNoiseParameters.setAngularAccelerationProcessNoiseStandardDeviation(1e-4);

      sensorNoiseParameters.setOrientationMeasurementStandardDeviation(Math.sqrt(1e-4));
      sensorNoiseParameters.setAngularVelocityMeasurementStandardDeviation(1e-4);
      sensorNoiseParameters.setLinearAccelerationMeasurementStandardDeviation(1e-4);

      sensorNoiseParameters.setAngularVelocityBiasProcessNoiseStandardDeviation(0.003);
      sensorNoiseParameters.setLinearAccelerationBiasProcessNoiseStandardDeviation(0.003);

      sensorNoiseParameters.setInitialLinearVelocityBias(new Vector3d());
      sensorNoiseParameters.setInitialAngularVelocityBias(new Vector3d());

      return sensorNoiseParameters;
   }

   public static SensorNoiseParameters createTunedNoiseParametersForEvaluator()
   {
      SensorNoiseParameters sensorNoiseParameters = new SensorNoiseParameters();

      sensorNoiseParameters.setComAccelerationProcessNoiseStandardDeviation(0.3);
      sensorNoiseParameters.setAngularAccelerationProcessNoiseStandardDeviation(0.3);

      sensorNoiseParameters.setOrientationMeasurementStandardDeviation(0.05);
      sensorNoiseParameters.setAngularVelocityMeasurementStandardDeviation(0.05);
      sensorNoiseParameters.setLinearAccelerationMeasurementStandardDeviation(1.0);

      sensorNoiseParameters.setAngularVelocityBiasProcessNoiseStandardDeviation(0.001);
      sensorNoiseParameters.setLinearAccelerationBiasProcessNoiseStandardDeviation(0.001);

      sensorNoiseParameters.setInitialLinearVelocityBias(new Vector3d());
      sensorNoiseParameters.setInitialAngularVelocityBias(new Vector3d());

      return sensorNoiseParameters;
   }

   public static SensorNoiseParameters createZeroNoiseParameters()
   {
      return new SensorNoiseParameters();
   }

}
