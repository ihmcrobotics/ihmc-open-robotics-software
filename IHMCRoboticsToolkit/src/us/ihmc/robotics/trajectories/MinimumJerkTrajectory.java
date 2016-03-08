package us.ihmc.robotics.trajectories;


public class MinimumJerkTrajectory
{
   private double C0, C1, C2, C3, C4, C5;
   private double moveDuration;
   private double timeInMove;
   private double position, velocity, acceleration;
   private boolean moveInitialized;

   public MinimumJerkTrajectory()
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

   /**
    * Calling this method will reset the timeInMove value from a previous move.
    * @param X0
    * @param V0
    * @param A0
    * @param Xf
    * @param Vf
    * @param Af
    * @param moveDuration
    */
   public void setMoveParameters(double X0, double V0, double A0, double Xf, double Vf, double Af, double moveDuration)
   {
      if (moveDuration <= 0.0)
         throw new RuntimeException("move Duration must be greater than 0.0");


      this.moveDuration = moveDuration;
      double DT = moveDuration;
      double DT2 = DT * DT;

      C0 = 1.0000 * X0;
      C1 = 1.0000 * V0 * DT;
      C2 = 0.5000 * A0 * DT2;
      C3 = -10.0000 * X0 - 6.0000 * V0 * DT - 1.5000 * A0 * DT2 + 10.0000 * Xf - 4.0000 * Vf * DT + 0.5000 * Af * DT2;
      C4 = 15.0000 * X0 + 8.0000 * V0 * DT + 1.5000 * A0 * DT2 - 15.0000 * Xf + 7.0000 * Vf * DT - 1.0000 * Af * DT2;
      C5 = -6.0000 * X0 - 3.0000 * V0 * DT - 0.5000 * A0 * DT2 + 6.0000 * Xf - 3.0000 * Vf * DT + 0.5000 * Af * DT2;

      this.timeInMove = 0.0;
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

      if (DT > 0.0)
         tau = timeInMove / moveDuration;
      else
         tau = 0.0;

      double tau2 = tau * tau;
      double tau3 = tau * tau2;
      double tau4 = tau * tau3;
      double tau5 = tau * tau4;

      position = C0 + C1 * tau + C2 * tau2 + C3 * tau3 + C4 * tau4 + C5 * tau5;
      velocity = (C1 + 2.0 * C2 * tau + 3.0 * C3 * tau2 + 4.0 * C4 * tau3 + 5.0 * C5 * tau4) / DT;
      acceleration = (2.0 * C2 + 6.0 * C3 * tau + 12.0 * C4 * tau2 + 20.0 * C5 * tau3) / DT2;
      this.timeInMove = timeInMove;
   }
}
