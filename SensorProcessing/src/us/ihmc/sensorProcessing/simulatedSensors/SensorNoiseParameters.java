package us.ihmc.sensorProcessing.simulatedSensors;

import javax.vecmath.Vector3d;

public class SensorNoiseParameters
{
   private double orientationMeasurementStandardDeviation = 0.0;
   private double angularVelocityMeasurementStandardDeviation = 0.0;
   private double linearAccelerationMeasurementStandardDeviation = 0.0;
   private double pointVelocityMeasurementStandardDeviation = 0.0;

   private double angularVelocityBiasProcessNoiseStandardDeviation = 0.0;
   private double linearAccelerationBiasProcessNoiseStandardDeviation = 0.0;

   private final Vector3d initialLinearVelocityBias = new Vector3d();
   private final Vector3d initialAngularVelocityBias = new Vector3d();

   public double getOrientationMeasurementStandardDeviation()
   {
      return orientationMeasurementStandardDeviation;
   }

   public double getAngularVelocityBiasProcessNoiseStandardDeviation()
   {
      return angularVelocityBiasProcessNoiseStandardDeviation;
   }

   public double getAngularVelocityMeasurementStandardDeviation()
   {
      return angularVelocityMeasurementStandardDeviation;
   }

   public void getInitialAngularVelocityBias(Vector3d vectorToPack)
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

   public void getInitialLinearVelocityBias(Vector3d vectorToPack)
   {
      vectorToPack.set(initialLinearVelocityBias);
   }

   public double getPointVelocityMeasurementStandardDeviation()
   {
      return pointVelocityMeasurementStandardDeviation;
   }

   public void setOrientationMeasurementStandardDeviation(double orientationMeasurementStandardDeviation)
   {
      this.orientationMeasurementStandardDeviation = orientationMeasurementStandardDeviation;
   }

   public void setAngularVelocityMeasurementStandardDeviation(double angularVelocityMeasurementStandardDeviation)
   {
      this.angularVelocityMeasurementStandardDeviation = angularVelocityMeasurementStandardDeviation;
   }

   public void setLinearAccelerationMeasurementStandardDeviation(double linearAccelerationMeasurementStandardDeviation)
   {
      this.linearAccelerationMeasurementStandardDeviation = linearAccelerationMeasurementStandardDeviation;
   }

   public void setPointVelocityMeasurementStandardDeviation(double pointVelocityMeasurementStandardDeviation)
   {
      this.pointVelocityMeasurementStandardDeviation = pointVelocityMeasurementStandardDeviation;
   }

   public void setAngularVelocityBiasProcessNoiseStandardDeviation(double angularVelocityBiasProcessNoiseStandardDeviation)
   {
      this.angularVelocityBiasProcessNoiseStandardDeviation = angularVelocityBiasProcessNoiseStandardDeviation;
   }

   public void setLinearAccelerationBiasProcessNoiseStandardDeviation(double linearAccelerationBiasProcessNoiseStandardDeviation)
   {
      this.linearAccelerationBiasProcessNoiseStandardDeviation = linearAccelerationBiasProcessNoiseStandardDeviation;
   }

   public double setPointVelocityMeasurementStandardDeviation()
   {
      return pointVelocityMeasurementStandardDeviation;
   }

   public void setInitialLinearVelocityBias(Vector3d initialLinearVelocityBias)
   {
      this.initialLinearVelocityBias.set(initialLinearVelocityBias);
   }

   public void setInitialAngularVelocityBias(Vector3d initialAngularVelocityBias)
   {
      this.initialAngularVelocityBias.set(initialAngularVelocityBias);
   }

}
