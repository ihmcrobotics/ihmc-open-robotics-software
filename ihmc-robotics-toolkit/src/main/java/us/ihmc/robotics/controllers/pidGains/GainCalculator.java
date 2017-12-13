package us.ihmc.robotics.controllers.pidGains;

public class GainCalculator
{
   /**
    * Assumes unit mass.
    */
   public static double computeDerivativeGain(double proportionalGain, double dampingRatio)
   {
      return 2.0 * dampingRatio * Math.sqrt(proportionalGain);
   }

   /**
    * Assumes unit mass.
    */
   public static double computeDampingRatio(double proportionalGain, double dampingGain)
   {
      return dampingGain / (2.0 * Math.sqrt(proportionalGain));
   }

   public static double computeDampingForSecondOrderSystem(double mass, double stiffness, double dampingRatio)
   {
      double criticalDamping = 2.0 * Math.sqrt(stiffness * mass);
      double damping = dampingRatio * criticalDamping;
      return damping;
   }
}
