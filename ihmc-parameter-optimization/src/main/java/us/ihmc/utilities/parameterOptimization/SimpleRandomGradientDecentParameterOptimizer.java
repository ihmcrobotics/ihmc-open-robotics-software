package us.ihmc.utilities.parameterOptimization;

import java.util.ArrayList;
import java.util.Random;

public class SimpleRandomGradientDecentParameterOptimizer implements ParameterOptimizer
{
   private ArrayList<EvaluatedIndividualListener> evaluatedIndividualListeners;

   private final double stepChange;
   private final Random random;
   
   public SimpleRandomGradientDecentParameterOptimizer(Random random, double stepChange)
   {
      this.random = random;
      this.stepChange = stepChange;
   }
   
   public IndividualToEvaluate optimize(OptimizationProblem optimizationProblem)
   {
      boolean maximize = optimizationProblem.getMaximize();
      double cutoffFitness = optimizationProblem.getCutoffFitness();
      
      IndividualToEvaluate seedIndividual = optimizationProblem.getSeedIndividualToEvaluate();
      ListOfParametersToOptimize seedParametersToOptimize = seedIndividual.getAllParametersToOptimize();
      int numberOfParameters = seedParametersToOptimize.getNumberOfParameters();
      
      double[] zeroToOnes = new double[numberOfParameters];
      
      for (int i=0; i<zeroToOnes.length; i++)
      {
         zeroToOnes[i] = random.nextDouble();
      }
      
      double bestCost;
      if (maximize) bestCost = Double.NEGATIVE_INFINITY;
      else bestCost = Double.POSITIVE_INFINITY;
      
      IndividualToEvaluate bestIndividual = null;
      int maximumNumberOfEvaluations = optimizationProblem.getMaximumNumberOfIndividualsToEvaluate();
      
      for (int i=0; i<maximumNumberOfEvaluations; i++)
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
         notifyEvaluatedIndividualListeners(testIndividual);
         
//         double cost = costFunction.evaluate(listOfParametersToOptimize);
//         System.out.println("Parameter optimizer: cost = " + cost + ", bestCost = " + bestCost);
         
         boolean improvement = isAnImprovement(cost, bestCost, maximize);
         
         if (improvement)
         {
            bestCost = cost;
            bestIndividual = testIndividual;
         }
         else
         {
            zeroToOnes[parameterToChangeIndex] = currentValue;
         }

         boolean cutoffFitnessReached = isAnImprovement(bestCost, cutoffFitness, maximize);
         if (cutoffFitnessReached) return bestIndividual;
      }
      
      return bestIndividual;
   }

   private boolean isAnImprovement(double newCost, double oldCost, boolean maximize)
   {
      if (maximize)
      {
         return newCost > oldCost;
      }
      else
      {
         return newCost < oldCost;
      }
   }

   
   private void notifyEvaluatedIndividualListeners(IndividualToEvaluate individual)
   {
      if (evaluatedIndividualListeners == null) return;
      
      for (EvaluatedIndividualListener listener : evaluatedIndividualListeners)
      {
         listener.evaluatedIndividual(individual);
      }
   }
   
   public void attachEvaluatedIndividualListener(EvaluatedIndividualListener listener)
   {
      if (evaluatedIndividualListeners == null) evaluatedIndividualListeners = new ArrayList<EvaluatedIndividualListener>();
      
      evaluatedIndividualListeners.add(listener);
   }

   public void createGUI()
   {
   }
   
}