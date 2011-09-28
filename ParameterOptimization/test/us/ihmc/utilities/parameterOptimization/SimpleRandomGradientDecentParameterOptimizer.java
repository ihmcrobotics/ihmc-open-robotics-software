package us.ihmc.utilities.parameterOptimization;

import java.util.Random;

import us.ihmc.utilities.parameterOptimization.IndividualToEvaluate;
import us.ihmc.utilities.parameterOptimization.ListOfParametersToOptimize;
import us.ihmc.utilities.parameterOptimization.OptimizationProblem;
import us.ihmc.utilities.parameterOptimization.ParameterOptimizer;

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
   
   public IndividualToEvaluate optimize(OptimizationProblem optimizationProblem)
   {
      IndividualToEvaluate seedIndividual = optimizationProblem.getSeedIndividualToEvaluate();
      ListOfParametersToOptimize seedParametersToOptimize = seedIndividual.getAllParametersToOptimize();
      int numberOfParameters = seedParametersToOptimize.getNumberOfParameters();
      
      double[] zeroToOnes = new double[numberOfParameters];
      
      for (int i=0; i<zeroToOnes.length; i++)
      {
         zeroToOnes[i] = random.nextDouble();
      }
      
      double bestCost = Double.POSITIVE_INFINITY;
      IndividualToEvaluate bestIndividual = null;
      
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
         
         IndividualToEvaluate testIndividual = seedIndividual.createNewIndividual();
         ListOfParametersToOptimize listOfParametersToOptimize = testIndividual.getAllParametersToOptimize();
         listOfParametersToOptimize.setCurrentValuesGivenZeroToOnes(zeroToOnes);

         testIndividual.startEvaluation();
         while(!testIndividual.isEvaluationDone())
         {
            try
            {
               Thread.sleep(100);
            } 
            catch (InterruptedException e)
            {
            }
         }
         double cost = testIndividual.getFitness();
         
//         double cost = costFunction.evaluate(listOfParametersToOptimize);
//         System.out.println("Parameter optimizer: cost = " + cost + ", bestCost = " + bestCost);
         
         if (cost < bestCost)
         {
            bestCost = cost;
            bestIndividual = testIndividual;
         }
         else
         {
            zeroToOnes[parameterToChangeIndex] = currentValue;
         }
      }
      
      return bestIndividual;
   }

   public void attachListener()
   {
      
   }
   
}