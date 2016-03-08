package us.ihmc.sensorProcessing.encoder.comparison;


public class ChirpEncoderProcessorEvaluationTrajectory implements EncoderProcessorEvaluationTrajectory
{
   private final double frequencyRate;
   private final double amplitude;
   
   private double startTime = Double.NaN;
   
   private double q;
   private double qd;

   public ChirpEncoderProcessorEvaluationTrajectory(double frequencyRate, double amplitude)
   {
      this.frequencyRate = frequencyRate;
      this.amplitude = amplitude;
   }

   public void update(double time)
   {
      if (Double.isNaN(startTime))
      {
         startTime = time;
      }
      
      double elapsedTime = startTime - time;
      
      q = amplitude * Math.sin(Math.PI * frequencyRate * elapsedTime * elapsedTime);
      qd = -amplitude * 2.0 * Math.PI * elapsedTime * frequencyRate * Math.cos(Math.PI * frequencyRate * elapsedTime * elapsedTime);
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
