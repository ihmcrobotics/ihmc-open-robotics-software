package us.ihmc.sensorProcessing.encoder.comparison;

public class SinusoidalEncoderProcessorEvaluationTrajectory implements EncoderProcessorEvaluationTrajectory
{
   private final double frequencyRadiansPerSecond;
   private final double amplitude;
   private final double phase;
   private double q;
   private double qd;

   public SinusoidalEncoderProcessorEvaluationTrajectory(double frequencyRadiansPerSecond, double amplitude, double phase)
   {
      this.frequencyRadiansPerSecond = frequencyRadiansPerSecond;
      this.amplitude = amplitude;
      this.phase = phase;
   }

   public void update(double time)
   {
      q = amplitude * Math.sin(frequencyRadiansPerSecond * time + phase);
      qd = frequencyRadiansPerSecond * amplitude * Math.cos(frequencyRadiansPerSecond * time + phase);
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
