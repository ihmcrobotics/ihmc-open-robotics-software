package us.ihmc.util.parameterOptimization;


public class OptimizationProblem
{
   private final IndividualToEvaluate seedIndividual;
   private final boolean maximize;
   private final double cutoffFitness;
   
   public OptimizationProblem(IndividualToEvaluate seedIndividual, boolean maximize, double cutoffFitness)
   {
      this.seedIndividual = seedIndividual;
      this.maximize = maximize;
      this.cutoffFitness = cutoffFitness;
   }
   
   public IndividualToEvaluate getSeedIndividualToEvaluate()
   {
      return seedIndividual;
   }
   
   public boolean getMaximize()
   {
      return maximize;
   }
   
   public double getCutoffFitness()
   {
      return cutoffFitness;
   }
}
