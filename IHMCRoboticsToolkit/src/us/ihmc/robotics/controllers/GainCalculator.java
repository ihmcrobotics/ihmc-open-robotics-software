package us.ihmc.robotics.controllers;

public class GainCalculator
{
   private GainCalculator()
   {
      // empty
   }

   /**
    * Assumes unit mass.
    */
   public static double computeDerivativeGain(double proportionalGain, double dampingRatio)
   {
      return 2.0 * dampingRatio * Math.sqrt(proportionalGain);
   }

   public static double computeDampingForSecondOrderSystem(double mass, double stiffness, double dampingRatio)
   {
      double criticalDamping = 2.0 * Math.sqrt(stiffness * mass);
      double damping = dampingRatio * criticalDamping;
      return damping;
   }
}
