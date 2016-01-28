package us.ihmc.aware.planning;


public class ParabolicTrajectory
{
   private double C0, C1, C2;
   private double moveDuration;
   private double timeInMove;
   private double position, velocity, acceleration;
   private boolean moveInitialized;

   public ParabolicTrajectory()
   {
      moveInitialized = false;
   }

   public double getPosition()
   {
      return position;
   }

   public double getVelocity()
   {
      return velocity;
   }

   public double getAcceleration()
   {
      return acceleration;
   }

   public double getMoveDuration()
   {
      return moveDuration;
   }

   public double getTimeInMove()
   {
      return timeInMove;
   }

   public void setMoveParameters(double X0, double Xm, double Xf, double moveDuration)
   {
      if (moveDuration <= 0.0)
         throw new RuntimeException("moveDuration must be greater than 0.0");

      this.moveDuration = moveDuration;

      C0 = 1.000 * X0;
      C1 =-1.000 * Xf + 4.000 * Xm - 3.000 * X0;
      C2 = 2.000 * Xf - 4.000 * Xm + 2.000 * X0;

      moveInitialized = true;
   }


   public void computeTrajectory(double timeInMove)
   {
      if (!moveInitialized)
         throw new RuntimeException("move must be initialized before computing trajectory");

      if (timeInMove < 0.0)
         timeInMove = 0.0;
      else if (timeInMove > moveDuration)
         timeInMove = moveDuration;

      double DT = moveDuration;
      double DT2 = DT * DT;

      double tau;
      if (moveDuration > 0.0)
         tau = timeInMove / moveDuration;
      else
         tau = 0.0;
      double tau2 = tau * tau;

      position = C0 + C1 * tau + C2 * tau2;
      velocity = (C1 + 2 * C2 * tau) / DT;
      acceleration = (2 * C2) / DT2;
      this.timeInMove = timeInMove;
   }
}
