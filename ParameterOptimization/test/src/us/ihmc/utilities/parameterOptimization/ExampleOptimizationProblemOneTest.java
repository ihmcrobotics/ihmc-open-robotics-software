package us.ihmc.utilities.parameterOptimization;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class ExampleOptimizationProblemOneTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSimpleXSquaredOptimization()
   {
      SimpleXSquaredCostFunction sampleCostFunction = new SimpleXSquaredCostFunction();      
      boolean maximize = false;
      double cutoffFitness = Double.NEGATIVE_INFINITY;
      
      int maximumNumberOfIndividualsToEvaluate = 500;
      OptimizationProblem optimizationProblem = new OptimizationProblem(sampleCostFunction, maximize, cutoffFitness, maximumNumberOfIndividualsToEvaluate);
      
      double stepChange = 0.01;
      Random random = new Random(1776L);
      
      SimpleRandomGradientDecentParameterOptimizer optimizer = new SimpleRandomGradientDecentParameterOptimizer(random, stepChange);
      IndividualToEvaluate optimalIndividualToEvaluate = optimizer.optimize(optimizationProblem);
      
      ListOfParametersToOptimize optimalListOfParametersToOptimize = optimalIndividualToEvaluate.getAllParametersToOptimize();
      DoubleParameterToOptimize optimalXParameter = (DoubleParameterToOptimize) optimalListOfParametersToOptimize.get(0);
      double optimalX = optimalXParameter.getCurrentValue();
      
      assertEquals(10.0, optimalX, 0.02);
   }
   
   private class SimpleXSquaredCostFunction extends IndividualToEvaluate
   {
      private double cost;
      private boolean isEvaluationDone = false;
      
      private final DoubleParameterToOptimize xToOptimize;
      private final ListOfParametersToOptimize listOfParametersToOptimize;
      
      public SimpleXSquaredCostFunction()
      {
         listOfParametersToOptimize = new ListOfParametersToOptimize();
         xToOptimize = new DoubleParameterToOptimize("x", 0.0, 100.0, listOfParametersToOptimize);
      }
      
      public ListOfParametersToOptimize getStructuralParametersToOptimize()
      {
         return null;
      }
      
      public ListOfParametersToOptimize getControlParametersToOptimize()
      {
         return listOfParametersToOptimize;
      }
      
      public void startEvaluation()
      {
         isEvaluationDone = false;
         
         
         double x = xToOptimize.getCurrentValue();
         
//         System.out.println("evaluate: x = " + x);
         cost = (x-10.0) * (x-10.0);
         isEvaluationDone = true;
      }

      public boolean isEvaluationDone()
      {
         return isEvaluationDone;
      }

      public double computeFitness()
      {
         return cost;
      }

      public IndividualToEvaluate createNewIndividual()
      {
         return new SimpleXSquaredCostFunction();
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSimpleThreeParameterCostFunction()
   {      
      SimpleThreeParameterCostFunction sampleCostFunction = new SimpleThreeParameterCostFunction();
      
      boolean maximize = false;
      double cutoffFitness = Double.NEGATIVE_INFINITY;
      int maximumNumberOfIndividualsToEvaluate = 2000;
      OptimizationProblem optimizationProblem = new OptimizationProblem(sampleCostFunction, maximize, cutoffFitness, maximumNumberOfIndividualsToEvaluate);
      
      double stepChange = 0.01;
      Random random = new Random(1776L);

      SimpleRandomGradientDecentParameterOptimizer optimizer = new SimpleRandomGradientDecentParameterOptimizer(random, stepChange);
      SimpleThreeParameterCostFunction optimalIndividual = (SimpleThreeParameterCostFunction) optimizer.optimize(optimizationProblem);
            
      ListOfParametersToOptimize listOfParametersToOptimize = optimalIndividual.getAllParametersToOptimize();
      DoubleParameterToOptimize xParameterToOptimize = (DoubleParameterToOptimize) listOfParametersToOptimize.get(0);
      BooleanParameterToOptimize booleanParameterToOptimize = (BooleanParameterToOptimize) listOfParametersToOptimize.get(1);
      IntegerParameterToOptimize integerParameterToOptimize = (IntegerParameterToOptimize) listOfParametersToOptimize.get(2);
      
      double optimalX = xParameterToOptimize.getCurrentValue();
      boolean optimalBoolean = booleanParameterToOptimize.getCurrentValue();
      int optimalInteger = integerParameterToOptimize.getCurrentValue();
      
      assertEquals(10.0, optimalX, 0.02);
      
      if (optimalBoolean)
      {
         assertEquals(-250.0, optimalInteger, 2.0);
      }
      else
      {
         assertEquals(250.0, optimalInteger, 2.0);
      }
   }
   
   private class SimpleThreeParameterCostFunction extends IndividualToEvaluate
   {
      private double cost;
      private boolean isEvaluationDone = false;
      
      private final ListOfParametersToOptimize listOfParametersToOptimize;
      
      private final DoubleParameterToOptimize xToOptimize;
      private final BooleanParameterToOptimize booleanToOptimize;
      private final IntegerParameterToOptimize integerToOptimize;
      
     
      public SimpleThreeParameterCostFunction()
      {
         listOfParametersToOptimize = new ListOfParametersToOptimize();
         
         xToOptimize = new DoubleParameterToOptimize("x", 0.0, 100.0, listOfParametersToOptimize);
         booleanToOptimize = new BooleanParameterToOptimize("boolean", listOfParametersToOptimize);
         integerToOptimize = new IntegerParameterToOptimize("integer", -250, 250, listOfParametersToOptimize);
      }
      
      public IndividualToEvaluate createNewIndividual()
      {
         return new SimpleThreeParameterCostFunction();
      }

      public void startEvaluation()
      {
         double candidateX = xToOptimize.getCurrentValue();
         boolean candidateBoolean = booleanToOptimize.getCurrentValue();
         int candidateInteger = integerToOptimize.getCurrentValue();

//         System.out.println("evaluate: candidateX = " + candidateX + ", candidateBoolean = " + candidateBoolean + ", candidateInteger = " + candidateInteger);
         
         if (candidateBoolean)
         {
            cost = candidateInteger + (10.0 - candidateX) * (10.0 - candidateX);
         }
         else
         {
            cost = -candidateInteger + (10.0 - candidateX) * (10.0 - candidateX);
         }
         
         isEvaluationDone = true;
      }

      public boolean isEvaluationDone()
      {
         return isEvaluationDone;
      }

      public double computeFitness()
      {
         return cost;
      }

      public ListOfParametersToOptimize getStructuralParametersToOptimize()
      {
         return null;
      }
      
      public ListOfParametersToOptimize getControlParametersToOptimize()
      {
         return listOfParametersToOptimize;
      }
   }
 
}
