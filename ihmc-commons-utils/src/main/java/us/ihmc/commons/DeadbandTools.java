package us.ihmc.commons;

public class DeadbandTools
{
   /**
    * Applies a deadband of size {@param deadbandSize} to value {@param value}, centered about zero.
    * <p>
    * For example, if a deadband of 1 is applied to a signal of 3, the return is 2. If a deadband of 3 is applied to a signal of 3, the return is 0.
    * </p>
    *
    * @param deadbandSize size of deadband to apply
    * @param value        value to apply the deadband to.
    * @return value after applying the deadband.
    */
   public static double applyDeadband(double deadbandSize, double value)
   {
      return applyDeadband(deadbandSize, 0.0, value);
   }

   /**
    * Applies a deadband of size {@param deadbandSize} to value {@param value}, centered about {@param deadbandCenter}.
    * <p>
    * For example, if a deadband of 1 is applied to a signal of 3 about 2, the return is 2. if a deadband of 3 is applied to a signal of 3 about 1, the return
    * is 1.
    * </p>
    *
    * @param deadbandSize   size of deadband to apply
    * @param deadbandCenter center about which the deadband is applied.
    * @param value          value to apply the deadband to.
    * @return value after applying the deadband.
    */
   public static double applyDeadband(double deadbandSize, double deadbandCenter, double value)
   {
      if (value > deadbandCenter)
         return Math.max(deadbandCenter, value - deadbandSize);
      else
         return Math.min(deadbandCenter, value + deadbandSize);
   }
}
