package us.ihmc.sensorProcessing.simulatedSensors;

import us.ihmc.euclid.tuple3D.Vector3D;

public class SensorNoiseParameters
{
   private double jointPositionMeasurementStandardDeviation = 0.0;
   private double jointVelocityMeasurementStandardDeviation = 0.0;

   private double comAccelerationProcessNoiseStandardDeviation = 0.0;
   private double angularAccelerationProcessNoiseStandardDeviation = 0.0;

   private double orientationMeasurementStandardDeviation = 0.0;
   private double orientationMeasurementLatency = 0.0;

   private double angularVelocityMeasurementStandardDeviation = 0.0;
   private double angularVelocityMeasurmentLatency = 0.0;

   private double linearAccelerationMeasurementStandardDeviation = 0.0;

   private double angularVelocityBiasProcessNoiseStandardDeviation = 0.0;
   private double linearAccelerationBiasProcessNoiseStandardDeviation = 0.0;

   private double imuYawDriftAcceleration = 0.0;

   private final Vector3D initialLinearVelocityBias = new Vector3D();
   private final Vector3D initialAngularVelocityBias = new Vector3D();

   public double getJointPositionMeasurementStandardDeviation()
   {
      return this.jointPositionMeasurementStandardDeviation;
   }

   public double getJointVelocityMeasurementStandardDeviation()
   {
      return this.jointVelocityMeasurementStandardDeviation;
   }

   public void setJointPositionMeasurementStandardDeviation(double jointPositionMeasurementStandardDeviation)
   {
      this.jointPositionMeasurementStandardDeviation = jointPositionMeasurementStandardDeviation;
   }

   public void setJointVelocityMeasurementStandardDeviation(double jointVelocityMeasurementStandardDeviation)
   {
      this.jointVelocityMeasurementStandardDeviation = jointVelocityMeasurementStandardDeviation;
   }

   public double getComAccelerationProcessNoiseStandardDeviation()
   {
      return comAccelerationProcessNoiseStandardDeviation;
   }

   public void setComAccelerationProcessNoiseStandardDeviation(double comAccelerationProcessNoiseStandardDeviation)
   {
      this.comAccelerationProcessNoiseStandardDeviation = comAccelerationProcessNoiseStandardDeviation;
   }

   public double getAngularAccelerationProcessNoiseStandardDeviation()
   {
      return angularAccelerationProcessNoiseStandardDeviation;
   }

   public void setAngularAccelerationProcessNoiseStandardDeviation(double angularAccelerationProcessNoiseStandardDeviation)
   {
      this.angularAccelerationProcessNoiseStandardDeviation = angularAccelerationProcessNoiseStandardDeviation;
   }

   public double getOrientationMeasurementStandardDeviation()
   {
      return orientationMeasurementStandardDeviation;
   }

   public double getOrientationMeasurementLatency()
   {
      return orientationMeasurementLatency;
   }

   public double getAngularVelocityBiasProcessNoiseStandardDeviation()
   {
      return angularVelocityBiasProcessNoiseStandardDeviation;
   }

   public double getAngularVelocityMeasurementStandardDeviation()
   {
      return angularVelocityMeasurementStandardDeviation;
   }

   public double getAngularVelocityMeasurementLatency()
   {
      return angularVelocityMeasurmentLatency;
   }

   public void getInitialAngularVelocityBias(Vector3D vectorToPack)
   {
      vectorToPack.set(initialAngularVelocityBias);
   }

   public double getLinearAccelerationMeasurementStandardDeviation()
   {
      return linearAccelerationMeasurementStandardDeviation;
   }

   public double getLinearAccelerationBiasProcessNoiseStandardDeviation()
   {
      return linearAccelerationBiasProcessNoiseStandardDeviation;
   }

   public void getInitialLinearVelocityBias(Vector3D vectorToPack)
   {
      vectorToPack.set(initialLinearVelocityBias);
   }

   public void setOrientationMeasurementStandardDeviation(double orientationMeasurementStandardDeviation)
   {
      this.orientationMeasurementStandardDeviation = orientationMeasurementStandardDeviation;
   }

   public void setOrientationMeasurementLatency(double orientationMeasurementLatency)
   {
      this.orientationMeasurementLatency = orientationMeasurementLatency;
   }

   public void setAngularVelocityMeasurementLatency(double angularVelocityMeasurmentLatency)
   {
      this.angularVelocityMeasurmentLatency = angularVelocityMeasurmentLatency;
   }

   public void setAngularVelocityMeasurementStandardDeviation(double angularVelocityMeasurementStandardDeviation)
   {
      this.angularVelocityMeasurementStandardDeviation = angularVelocityMeasurementStandardDeviation;
   }

   public void setLinearAccelerationMeasurementStandardDeviation(double linearAccelerationMeasurementStandardDeviation)
   {
      this.linearAccelerationMeasurementStandardDeviation = linearAccelerationMeasurementStandardDeviation;
   }

   public void setAngularVelocityBiasProcessNoiseStandardDeviation(double angularVelocityBiasProcessNoiseStandardDeviation)
   {
      this.angularVelocityBiasProcessNoiseStandardDeviation = angularVelocityBiasProcessNoiseStandardDeviation;
   }

   public void setLinearAccelerationBiasProcessNoiseStandardDeviation(double linearAccelerationBiasProcessNoiseStandardDeviation)
   {
      this.linearAccelerationBiasProcessNoiseStandardDeviation = linearAccelerationBiasProcessNoiseStandardDeviation;
   }

   public void setInitialLinearVelocityBias(Vector3D initialLinearVelocityBias)
   {
      this.initialLinearVelocityBias.set(initialLinearVelocityBias);
   }

   public void setInitialAngularVelocityBias(Vector3D initialAngularVelocityBias)
   {
      this.initialAngularVelocityBias.set(initialAngularVelocityBias);
   }

   public void setIMUYawDriftAcceleration(double yawDriftAcceleration)
   {
      this.imuYawDriftAcceleration = yawDriftAcceleration;
   }

   public double getIMUYawDriftAcceleration()
   {
      return this.imuYawDriftAcceleration;
   }
}
