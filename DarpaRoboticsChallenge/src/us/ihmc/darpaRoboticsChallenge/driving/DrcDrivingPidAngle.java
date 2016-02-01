package us.ihmc.darpaRoboticsChallenge.driving;

/**
 * @author Peter Abeles
 */
public class DrcDrivingPidAngle
{
   // proportional gain
   double Kp;
   // integral gain
   double Ki;
   // derivative gain
   double Kd;

   double integral;
   double previous;

   /**
    *
    * @param kp proportional gain
    * @param ki integral gain
    * @param kd derivative gain
    */
   public DrcDrivingPidAngle(double kp, double ki, double kd)
   {
      Kp = kp;
      Ki = ki;
      Kd = kd;
   }

   /**
    *
    * @param turnDirection Output turn direction from basic control algorithm -1 to 1 for turning left to right
    * @return
    */
   public double proccess( double turnDirection ) {
      if( Double.isNaN(turnDirection))
         return turnDirection;

      integral += turnDirection;

      double derivative = turnDirection - previous;

      previous = turnDirection;

      double out = Kp*turnDirection + Ki*integral + Kd*derivative;

      if( out > 1 )
         return 1;
      else if( out < -1 )
         return -1;

      return out;
   }


}
