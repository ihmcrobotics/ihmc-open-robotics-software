package us.ihmc.sensorProcessing.encoder.comparison;

import java.util.Random;

public class HoverATickEncoderProcessorEvaluationTrajectory implements EncoderProcessorEvaluationTrajectory
{
   private Random random = new Random(1123L);
   
   private double q;
   private double qd;
   private final double encoderTicksPerPosition;

   public HoverATickEncoderProcessorEvaluationTrajectory(double encoderTicksPerPosition)
   {
      this.encoderTicksPerPosition = encoderTicksPerPosition;
   }

   public void update(double time)
   {
      q = 0.9 / encoderTicksPerPosition * (0.5 - random.nextDouble());
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
