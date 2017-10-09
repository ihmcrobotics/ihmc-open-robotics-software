package us.ihmc.robotics.lidar;

public class SimulatedLIDARSensorLimitationParameters
{
   private double maxRange = Double.NaN;
   private double minRange = Double.NaN;
   private double quantization = Double.NaN;

   public double getMaxRange()
   {
      return maxRange;
   }

   public double getMinRange()
   {
      return minRange;
   }

   public void setMaxRange(double maxRange)
   {
      this.maxRange = maxRange;
   }

   public void setMinRange(double minRange)
   {
      this.minRange = minRange;
   }

   public double getQuantization()
   {
      return quantization;
   }

   public void setQuantization(double quantization)
   {
      this.quantization = quantization;
   }
}
