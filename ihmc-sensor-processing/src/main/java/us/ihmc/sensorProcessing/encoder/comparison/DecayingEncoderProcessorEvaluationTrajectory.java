package us.ihmc.sensorProcessing.encoder.comparison;

public class DecayingEncoderProcessorEvaluationTrajectory implements EncoderProcessorEvaluationTrajectory
{
   private final double frequencyRadiansPerSecond;
   private final double amplitude;
   private final double phase;
   private final double timeConstant;
   private double q;
   private double qd;

   public DecayingEncoderProcessorEvaluationTrajectory(double frequencyRadiansPerSecond, double amplitude, double phase, double timeConstant)
   {
      this.frequencyRadiansPerSecond = frequencyRadiansPerSecond;
      this.amplitude = amplitude;
      this.phase = phase;
      this.timeConstant = timeConstant;
   }

   public void update(double time)
   {
      double exp = Math.exp(-time / timeConstant);
      double sin = amplitude * Math.sin(frequencyRadiansPerSecond * time + phase);
      double cos = amplitude * Math.cos(frequencyRadiansPerSecond * time + phase);
      q = exp * sin;
      qd = -1.0 / timeConstant * exp * sin + exp * frequencyRadiansPerSecond * cos;
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
