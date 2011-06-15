package us.ihmc.sensorProcessing.encoder.comparison;

public class HoverATickEncoderProcessorEvaluationTrajectory implements EncoderProcessorEvaluationTrajectory
{
   private double q;
   private double qd;
   private final double encoderTicksPerPosition;

   public HoverATickEncoderProcessorEvaluationTrajectory(double encoderTicksPerPosition)
   {
      this.encoderTicksPerPosition = encoderTicksPerPosition;
   }

   public void update(double time)
   {
      q = 0.9 / encoderTicksPerPosition * (0.5 - Math.random());
      qd = 0;
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
