package us.ihmc.footstepPlanning.postProcessing;

public class SplitFractionTools
{
   static double appendSplitFraction(double desiredSplitFraction, double currentSplitFraction, double nominalSplitFraction)
   {
      if (currentSplitFraction == -1.0)
         return desiredSplitFraction;

      return desiredSplitFraction * currentSplitFraction / nominalSplitFraction;
   }

   static double appendWeightDistribution(double desiredWeightDistribution, double currentWeightDistribution, double nominalWeightDistribution)
   {
      if (currentWeightDistribution == -1.0)
         return desiredWeightDistribution;

      return desiredWeightDistribution * currentWeightDistribution / nominalWeightDistribution;
   }
}
