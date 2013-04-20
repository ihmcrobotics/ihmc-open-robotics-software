package us.ihmc.sensorProcessing.simulatedSensors;

import javax.vecmath.Vector3d;

public class SensorNoiseParameters
{
   private double comAccelerationProcessNoiseStandardDeviation = 0.0;
   private double angularAccelerationProcessNoiseStandardDeviation = 0.0;

   private double orientationMeasurementStandardDeviation = 0.0;
   private double angularVelocityMeasurementStandardDeviation = 0.0;
   private double linearAccelerationMeasurementStandardDeviation = 0.0;
   
   private double pointPositionMeasurementStandardDeviation = 0.0;
   private double pointVelocityMeasurementStandardDeviation = 0.0;

   private double angularVelocityBiasProcessNoiseStandardDeviation = 0.0;
   private double linearAccelerationBiasProcessNoiseStandardDeviation = 0.0;

   private final Vector3d initialLinearVelocityBias = new Vector3d();
   private final Vector3d initialAngularVelocityBias = new Vector3d();

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

   public double getPointPositionMeasurementStandardDeviation()
   {
      return pointPositionMeasurementStandardDeviation;
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

   public void setPointPositionMeasurementStandardDeviation(double pointPositionMeasurementStandardDeviation)
   {
      this.pointPositionMeasurementStandardDeviation = pointPositionMeasurementStandardDeviation;
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

   public void setInitialLinearVelocityBias(Vector3d initialLinearVelocityBias)
   {
      this.initialLinearVelocityBias.set(initialLinearVelocityBias);
   }

   public void setInitialAngularVelocityBias(Vector3d initialAngularVelocityBias)
   {
      this.initialAngularVelocityBias.set(initialAngularVelocityBias);
   }

}
