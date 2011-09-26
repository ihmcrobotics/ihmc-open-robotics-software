package us.ihmc.util.parameterOptimization;

import java.util.Random;

public class SimpleRandomGradientDecentParameterOptimizer implements ParameterOptimizer
{
   private final double stepChange;
   private final Random random = new Random();
   private final int numberOfEvaluations;

   public SimpleRandomGradientDecentParameterOptimizer(double stepChange, int numberOfEvaluations)
   {
      this.stepChange = stepChange;
      this.numberOfEvaluations = numberOfEvaluations;
   }
   
   public ListOfParametersToOptimize optimize(OptimizationProblem optimizationProblem)
   {
      CostFunction costFunction = optimizationProblem.getCostFunction();
      ListOfParametersToOptimize listOfParametersToOptimize = optimizationProblem.getListOfParametersToOptimize();
      int numberOfParameters = listOfParametersToOptimize.getNumberOfParameters();
      
      double[] zeroToOnes = new double[numberOfParameters];
      
      for (int i=0; i<zeroToOnes.length; i++)
      {
         zeroToOnes[i] = random.nextDouble();
      }
      
      double bestCost = Double.POSITIVE_INFINITY;
      
      for (int i=0; i<numberOfEvaluations; i++)
      {
         int parameterToChangeIndex = random.nextInt(numberOfParameters);
//         ParameterToOptimize parameterToChange = listOfParametersToOptimize.get(parameterToChangeIndex);
         double currentValue = zeroToOnes[parameterToChangeIndex];
         double change = (1.0 - 2.0 * random.nextDouble()) * stepChange;
         double newValue = currentValue + change;
         
         if (newValue < 0.0) newValue = 0.0;
         if (newValue > 1.0) newValue = 1.0;
         
         zeroToOnes[parameterToChangeIndex] = newValue;
         
         listOfParametersToOptimize.setCurrentValuesGivenZeroToOnes(zeroToOnes);

         double cost = costFunction.evaluate(listOfParametersToOptimize);
//         System.out.println("Parameter optimizer: cost = " + cost + ", bestCost = " + bestCost);
         
         if (cost < bestCost)
         {
            bestCost = cost;
         }
         else
         {
            zeroToOnes[parameterToChangeIndex] = currentValue;
         }
      }
      
      listOfParametersToOptimize.setCurrentValuesGivenZeroToOnes(zeroToOnes);
      return listOfParametersToOptimize;
   }

   public void attachListener()
   {
      
   }
   
}