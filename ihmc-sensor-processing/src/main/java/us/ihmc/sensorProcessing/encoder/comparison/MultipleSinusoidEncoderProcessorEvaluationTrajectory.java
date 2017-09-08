package us.ihmc.sensorProcessing.encoder.comparison;

public class MultipleSinusoidEncoderProcessorEvaluationTrajectory implements EncoderProcessorEvaluationTrajectory
{
   private final double[] frequencyRadiansPerSecond;
   private final double[] amplitude;
   private final double[] phase;
   private double q;
   private double qd;

   public MultipleSinusoidEncoderProcessorEvaluationTrajectory(double[] frequencyRadiansPerSecond, double[] amplitude, double[] phase)
   {
      this.frequencyRadiansPerSecond = frequencyRadiansPerSecond;
      this.amplitude = amplitude;
      this.phase = phase;
   }

   public void update(double time)
   {
      q = 0.0;
      qd = 0.0;

      for (int i = 0; i < frequencyRadiansPerSecond.length; i++)
      {
         q += amplitude[i] * Math.sin(frequencyRadiansPerSecond[i] * time + phase[i]);
         qd += frequencyRadiansPerSecond[i] * amplitude[i] * Math.cos(frequencyRadiansPerSecond[i] * time + phase[i]);
      }
   }

   public double getPosition()
   {
      return q;
   }

   public double getVelocity()
   {
      return qd;
   }
}
