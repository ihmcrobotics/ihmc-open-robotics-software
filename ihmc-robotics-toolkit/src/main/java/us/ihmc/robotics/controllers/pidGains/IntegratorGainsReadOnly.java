package us.ihmc.robotics.controllers.pidGains;

public interface IntegratorGainsReadOnly
{
   /**
    * Get the integral gain, as in a PID controller.
    * @return the integral gain
    */
   default double getKi()
   {
      return 0.0;
   }

   /**
    * Get the maximum total integral error. This can be used to limit the
    * maximum feedback action caused by the integration.
    * <p>
    * To disable integral error limiting set to {@link Double#POSITIVE_INFINITY}
    * </p>
    * @return the maximum integral error
    */
   default double getMaxIntegralError()
   {
      return Double.POSITIVE_INFINITY;
   }

   /**
    * Get the leak ratio of an integrator. If using a leaking integrator
    * the integal error will decay over time as it is multiplied by the leak
    * rate at each control tick.
    * <p>
    * To disable leaking set to {@code 1.0}
    * </p>
    * @return the integrator leak rate
    */
   default double getIntegralLeakRatio()
   {
      return 1.0;
   }
}
