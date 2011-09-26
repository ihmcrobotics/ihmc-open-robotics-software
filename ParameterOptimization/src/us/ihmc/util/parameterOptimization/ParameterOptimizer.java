package us.ihmc.util.parameterOptimization;

public interface ParameterOptimizer
{

   public abstract double[] optimize(OptimizationProblem optimizationProblem);
   public abstract void attachListener();
   
}
