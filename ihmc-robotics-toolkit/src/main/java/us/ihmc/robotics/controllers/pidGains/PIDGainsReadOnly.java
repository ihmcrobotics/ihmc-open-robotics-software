package us.ihmc.robotics.controllers.pidGains;

public interface PIDGainsReadOnly extends PDGainsReadOnly
{
   /**
    * Get the integral gain for a PID controller.
    * @return the integral gain
    */
   public abstract double getKi();

   /**
    * Get the maximum total integral error. This can be used to limit the
    * maximum feedback action caused by the integration.
    * <p>
    * To disable integral error limiting set to {@link Double#POSITIVE_INFINITY}
    * </p>
    * @return the maximum integral error
    */
   public abstract double getMaxIntegralError();

   /**
    * Get the leak ratio of an integrator. If using a leaking integrator
    * the integal error will decay over time as it is multiplied by the leak
    * rate at each control tick.
    * <p>
    * To disable leaking set to {@code 1.0}
    * </p>
    * @return the integrator leak rate
    */
   public abstract double getIntegralLeakRatio();
}
