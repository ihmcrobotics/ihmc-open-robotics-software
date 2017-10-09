package us.ihmc.sensorProcessing.encoder.comparison;

public class SawtoothEncoderProcessorEvaluationTrajectory implements EncoderProcessorEvaluationTrajectory
{
   private final double speed;
   private final double switchTime;
   private double q;
   private double qd;
   
   private double lastTime = 0.0;

   public SawtoothEncoderProcessorEvaluationTrajectory(double speed, double switchTime)
   {
      this.speed = speed;
      this.switchTime = switchTime;
   }

   public void update(double time)
   {
      int foo = (int) Math.round(time/switchTime);
      
      if (foo % 2 == 0)
      {
         qd = speed;
      }
      else
      {
         qd = -speed;
      }
      q = q + qd * (time - lastTime);
      
      lastTime = time;
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
