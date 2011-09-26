package us.ihmc.util.parameterOptimization;

public interface ParameterOptimizer
{

   public abstract ListOfParametersToOptimize optimize(OptimizationProblem optimizationProblem);
   public abstract void attachListener();
   
}
