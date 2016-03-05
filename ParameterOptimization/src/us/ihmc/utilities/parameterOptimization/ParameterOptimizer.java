package us.ihmc.utilities.parameterOptimization;

public interface ParameterOptimizer
{

   public abstract IndividualToEvaluate optimize(OptimizationProblem optimizationProblem);
   public abstract void attachEvaluatedIndividualListener(EvaluatedIndividualListener listener);
   public abstract void createGUI();
}
