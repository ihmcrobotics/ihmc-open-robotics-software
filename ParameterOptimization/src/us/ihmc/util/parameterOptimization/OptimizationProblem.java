package us.ihmc.util.parameterOptimization;


public class OptimizationProblem
{
   private final ListOfParametersToOptimize listOfParametersToOptimize;
   private final CostFunction costFunction;
   
   public OptimizationProblem(ListOfParametersToOptimize listOfParametersToOptimize, CostFunction costFunction)
   {
      this.listOfParametersToOptimize = listOfParametersToOptimize;
      this.costFunction = costFunction;
   }
   
   public ListOfParametersToOptimize getListOfParametersToOptimize()
   {
      return listOfParametersToOptimize;
   }
   
   public CostFunction getCostFunction()
   {
      return costFunction;
   }
}
