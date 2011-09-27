package us.ihmc.util.parameterOptimization;

public interface ParameterOptimizer
{

   public abstract IndividualToEvaluate optimize(OptimizationProblem optimizationProblem);
   public abstract void attachListener();
   
}
