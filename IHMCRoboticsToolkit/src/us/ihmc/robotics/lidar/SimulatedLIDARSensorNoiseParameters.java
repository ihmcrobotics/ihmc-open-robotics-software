package us.ihmc.robotics.lidar;

public class SimulatedLIDARSensorNoiseParameters
{
   private double gaussianNoiseStandardDeviation = Double.NaN;
   private double gaussianNoiseMean = Double.NaN;

   public double getGaussianNoiseStandardDeviation()
   {
      return gaussianNoiseStandardDeviation;
   }

   public void setGaussianNoiseStandardDeviation(double gaussianNoiseStandardDeviation)
   {
      this.gaussianNoiseStandardDeviation = gaussianNoiseStandardDeviation;
   }

   public double getGaussianNoiseMean()
   {
      return gaussianNoiseMean;
   }

   public void setGaussianNoiseMean(double gaussianNoiseMean)
   {
      this.gaussianNoiseMean = gaussianNoiseMean;
   }
}
