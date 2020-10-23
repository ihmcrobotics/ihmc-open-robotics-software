package us.ihmc.robotics.functionApproximation;

import us.ihmc.commons.MathTools;

public class ProbabilityDensityFunction
{
   private final static double SQRT_TAU = Math.sqrt(2.0 * Math.PI);

   public static double getProbabilityUsingNormalDistribution(double value, double mean, double standardDeviation)
   {
      double exponential = -0.5 * MathTools.square((value - mean) / standardDeviation);
      double probability = Math.exp(exponential);
      probability /= (standardDeviation * SQRT_TAU);
      return probability;
   }
}
