package us.ihmc.robotics.math;

public class DeadbandTools
{
   public static double applyDeadband(double deadbandSize, double value)
   {
      return applyDeadband(deadbandSize, 0.0, value);
   }

   public static double applyDeadband(double deadbandSize, double deadbandCenter, double value)
   {
      if (value > deadbandCenter)
         return Math.max(deadbandCenter, value - deadbandSize);
      else
         return Math.min(deadbandCenter, value + deadbandSize);
   }
}
