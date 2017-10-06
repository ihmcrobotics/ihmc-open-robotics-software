package us.ihmc.utilities.parameterOptimization;


public class OptimizationProblem
{
   private final IndividualToEvaluate seedIndividual;
   private final boolean maximize;
   private final double cutoffFitness;
   private final int maximumNumberOfIndividualsToEvaluate;
   
   public OptimizationProblem(IndividualToEvaluate seedIndividual, boolean maximize, double cutoffFitness, int maximumNumberOfIndividualsToEvaluate)
   {
      this.seedIndividual = seedIndividual;
      this.maximize = maximize;
      this.cutoffFitness = cutoffFitness;
      this.maximumNumberOfIndividualsToEvaluate = maximumNumberOfIndividualsToEvaluate;
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

   public int getMaximumNumberOfIndividualsToEvaluate()
   {
      return maximumNumberOfIndividualsToEvaluate;
   }
   
   
}
