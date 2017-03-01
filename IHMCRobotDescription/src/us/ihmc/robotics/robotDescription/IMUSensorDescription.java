package us.ihmc.robotics.robotDescription;

import us.ihmc.euclid.transform.RigidBodyTransform;

public class IMUSensorDescription extends SensorDescription
{
   private double accelerationNoiseMean;
   private double accelerationNoiseStandardDeviation;
   private double accelerationBiasMean;
   private double accelerationBiasStandardDeviation;

   private double angularVelocityNoiseMean;
   private double angularVelocityNoiseStandardDeviation;
   private double angularVelocityBiasMean;
   private double angularVelocityBiasStandardDeviation;

   public IMUSensorDescription(String name, RigidBodyTransform imuTransform)
   {
      super(name, imuTransform);
   }

   public void setAccelerationNoiseParameters(double noiseMean, double noiseStandardDeviation)
   {
      this.setAccelerationNoiseMean(noiseMean);
      this.setAccelerationNoiseStandardDeviation(noiseStandardDeviation);
   }

   public void setAccelerationBiasParameters(double biasMean, double biasStandardDeviation)
   {
      this.setAccelerationBiasMean(biasMean);
      this.setAccelerationBiasStandardDeviation(biasStandardDeviation);
   }

   public void setAngularVelocityNoiseParameters(double noiseMean, double noiseStandardDeviation)
   {
      this.setAngularVelocityNoiseMean(noiseMean);
      this.setAngularVelocityNoiseStandardDeviation(noiseStandardDeviation);
   }

   public void setAngularVelocityBiasParameters(double biasMean, double biasStandardDeviation)
   {
      this.setAngularVelocityNoiseMean(biasMean);
      this.setAngularVelocityNoiseStandardDeviation(biasStandardDeviation);
   }

   public double getAccelerationNoiseMean()
   {
      return accelerationNoiseMean;
   }

   public void setAccelerationNoiseMean(double accelerationNoiseMean)
   {
      this.accelerationNoiseMean = accelerationNoiseMean;
   }

   public double getAccelerationNoiseStandardDeviation()
   {
      return accelerationNoiseStandardDeviation;
   }

   public void setAccelerationNoiseStandardDeviation(double accelerationNoiseStandardDeviation)
   {
      this.accelerationNoiseStandardDeviation = accelerationNoiseStandardDeviation;
   }

   public double getAccelerationBiasMean()
   {
      return accelerationBiasMean;
   }

   public void setAccelerationBiasMean(double accelerationBiasMean)
   {
      this.accelerationBiasMean = accelerationBiasMean;
   }

   public double getAccelerationBiasStandardDeviation()
   {
      return accelerationBiasStandardDeviation;
   }

   public void setAccelerationBiasStandardDeviation(double accelerationBiasStandardDeviation)
   {
      this.accelerationBiasStandardDeviation = accelerationBiasStandardDeviation;
   }

   public double getAngularVelocityNoiseMean()
   {
      return angularVelocityNoiseMean;
   }

   public void setAngularVelocityNoiseMean(double angularVelocityNoiseMean)
   {
      this.angularVelocityNoiseMean = angularVelocityNoiseMean;
   }

   public double getAngularVelocityNoiseStandardDeviation()
   {
      return angularVelocityNoiseStandardDeviation;
   }

   public void setAngularVelocityNoiseStandardDeviation(double angularVelocityNoiseStandardDeviation)
   {
      this.angularVelocityNoiseStandardDeviation = angularVelocityNoiseStandardDeviation;
   }

   public double getAngularVelocityBiasMean()
   {
      return angularVelocityBiasMean;
   }

   public void setAngularVelocityBiasMean(double angularVelocityBiasMean)
   {
      this.angularVelocityBiasMean = angularVelocityBiasMean;
   }

   public double getAngularVelocityBiasStandardDeviation()
   {
      return angularVelocityBiasStandardDeviation;
   }

   public void setAngularVelocityBiasStandardDeviation(double angularVelocityBiasStandardDeviation)
   {
      this.angularVelocityBiasStandardDeviation = angularVelocityBiasStandardDeviation;
   }

}
