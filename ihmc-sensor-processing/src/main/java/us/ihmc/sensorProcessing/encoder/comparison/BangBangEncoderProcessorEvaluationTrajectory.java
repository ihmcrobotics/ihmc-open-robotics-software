package us.ihmc.sensorProcessing.encoder.comparison;


public class BangBangEncoderProcessorEvaluationTrajectory implements EncoderProcessorEvaluationTrajectory
{
   private final double maxAccel;
   private final double switchTime, dwellTime;
   private double q;
   private double qd;

   private double lastTime = Double.NaN;
   private double startTime;
   
   public BangBangEncoderProcessorEvaluationTrajectory(double maxAccel, double switchTime, double dwellTime)
   {
      this.maxAccel = maxAccel;
      this.switchTime = switchTime;
      this.dwellTime = dwellTime;
   }

   public void update(double time)
   {
      if (Double.isNaN(lastTime))
      {
         startTime = time;
         lastTime = time;
         qd = 0.0;
      }
      
      double elapsedTime = time-startTime + (switchTime - dwellTime)/2.0;
      int foo = (int) Math.round(elapsedTime / switchTime);
      double elapsedTimeThisTurn = elapsedTime % switchTime;
      
      if (foo % 2 == 0)
      {
         if (elapsedTimeThisTurn < switchTime - dwellTime)
            qd += maxAccel * (time - lastTime);
      }
      else
      {
         if (elapsedTimeThisTurn < switchTime - dwellTime)
            qd -= maxAccel * (time - lastTime);
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
