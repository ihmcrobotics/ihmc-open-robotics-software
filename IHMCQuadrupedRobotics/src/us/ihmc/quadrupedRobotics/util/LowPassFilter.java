package us.ihmc.quadrupedRobotics.util;

public class LowPassFilter
{
   private final double dt;
   private double breakFrequency;
   private double output;

   /**
    * backwards euler approximation of a first order low pass filter
    */
   public LowPassFilter(double dt, double breakFrequency)
   {
      this.dt = dt;
      this.breakFrequency = breakFrequency;
      this.output = 0;
   }

   public void setBreakFrequency(double breakFrequency)
   {
      this.breakFrequency = breakFrequency;
   }

   public void initialize(double output)
   {
      this.output = output;
   }

   public double compute(double input)
   {
      double alpha = 1.0 - 1.0 / (1.0 + 2.0 * Math.PI * breakFrequency * dt);
      return output = alpha * input + (1 - alpha) * output;
   }
}
