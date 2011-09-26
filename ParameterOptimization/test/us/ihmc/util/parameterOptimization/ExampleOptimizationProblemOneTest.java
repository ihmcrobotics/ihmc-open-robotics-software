package us.ihmc.util.parameterOptimization;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class ExampleOptimizationProblemOneTest
{

   @Test
   public void testSimpleXSquaredOptimization()
   {
      SimpleXSquaredCostFunction sampleCostFunction = new SimpleXSquaredCostFunction();
      ListOfParametersToOptimize listOfParametersToOptimize = new ListOfParametersToOptimize();
      
      DoubleParameterToOptimize xParameterToOptimize = new DoubleParameterToOptimize(0.0, 100.0);
      listOfParametersToOptimize.addParameterToOptimize(xParameterToOptimize);
      
      OptimizationProblem optimizationProblem = new OptimizationProblem(listOfParametersToOptimize, sampleCostFunction);
      
      double stepChange = 0.01;
      int numberOfEvaluations = 500;
      SimpleRandomGradientDecentParameterOptimizer optimizer = new SimpleRandomGradientDecentParameterOptimizer(stepChange, numberOfEvaluations);
      listOfParametersToOptimize = optimizer.optimize(optimizationProblem);
      
      double optimalX = xParameterToOptimize.getCurrentValue();
      
      assertEquals(10.0, optimalX, 0.02);
   }
   
   private class SimpleXSquaredCostFunction implements CostFunction
   {
      public double evaluate(ListOfParametersToOptimize listOfParameters)
      {
           DoubleParameterToOptimize xToOptimize = (DoubleParameterToOptimize) listOfParameters.get(0);
           
           double x = xToOptimize.getCurrentValue();
           
//           System.out.println("evaluate: x = " + x);
           return (x-10.0) * (x-10.0);
      }
   }
   
   
   @Test
   public void testSimpleThreeParameterCostFunction()
   {
      SimpleThreeParameterCostFunction sampleCostFunction = new SimpleThreeParameterCostFunction();
      ListOfParametersToOptimize listOfParametersToOptimize = new ListOfParametersToOptimize();
      
      DoubleParameterToOptimize xParameterToOptimize = new DoubleParameterToOptimize(0.0, 100.0);
      listOfParametersToOptimize.addParameterToOptimize(xParameterToOptimize);
      
      BooleanParameterToOptimize booleanParameterToOptimize = new BooleanParameterToOptimize();
      listOfParametersToOptimize.addParameterToOptimize(booleanParameterToOptimize);
      
      IntegerParameterToOptimize integerParameterToOptimize = new IntegerParameterToOptimize(-250, 250);
      listOfParametersToOptimize.addParameterToOptimize(integerParameterToOptimize);

      OptimizationProblem optimizationProblem = new OptimizationProblem(listOfParametersToOptimize, sampleCostFunction);
      
      double stepChange = 0.01;
      int numberOfEvaluations = 2000;
      SimpleRandomGradientDecentParameterOptimizer optimizer = new SimpleRandomGradientDecentParameterOptimizer(stepChange, numberOfEvaluations);
      listOfParametersToOptimize = optimizer.optimize(optimizationProblem);
            
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
   
   private class SimpleThreeParameterCostFunction implements CostFunction
   {
      public double evaluate(ListOfParametersToOptimize listOfParameters)
      {         
         DoubleParameterToOptimize xToOptimize = (DoubleParameterToOptimize) listOfParameters.get(0);
         BooleanParameterToOptimize booleanToOptimize = (BooleanParameterToOptimize) listOfParameters.get(1);
         IntegerParameterToOptimize integerToOptimize = (IntegerParameterToOptimize) listOfParameters.get(2);

         double candidateX = xToOptimize.getCurrentValue();
         boolean candidateBoolean = booleanToOptimize.getCurrentValue();
         int candidateInteger = integerToOptimize.getCurrentValue();

//         System.out.println("evaluate: candidateX = " + candidateX + ", candidateBoolean = " + candidateBoolean + ", candidateInteger = " + candidateInteger);
         
         if (candidateBoolean)
         {
            return candidateInteger + (10.0 - candidateX) * (10.0 - candidateX);
         }
         else
         {
            return -candidateInteger + (10.0 - candidateX) * (10.0 - candidateX);
         }

      }
   }
   
   
   
   @Test
   public void testEmbeddedCostFunction()
   {
      ListOfParametersToOptimize listOfParametersToOptimize = new ListOfParametersToOptimize();

      final DoubleParameterToOptimize xParameterToOptimize = new DoubleParameterToOptimize(0.0, 100.0);
      listOfParametersToOptimize.addParameterToOptimize(xParameterToOptimize);

      final BooleanParameterToOptimize booleanParameterToOptimize = new BooleanParameterToOptimize();
      listOfParametersToOptimize.addParameterToOptimize(booleanParameterToOptimize);

      final IntegerParameterToOptimize integerParameterToOptimize = new IntegerParameterToOptimize(-250, 250);
      listOfParametersToOptimize.addParameterToOptimize(integerParameterToOptimize);

      CostFunction sampleCostFunction = new CostFunction()
      {
         public double evaluate(ListOfParametersToOptimize listOfParameters)
         {
            double candidateX = xParameterToOptimize.getCurrentValue();
            boolean candidateBoolean = booleanParameterToOptimize.getCurrentValue();
            int candidateInteger = integerParameterToOptimize.getCurrentValue();

            // System.out.println("evaluate: candidateX = " + candidateX + ", candidateBoolean = " + candidateBoolean + ", candidateInteger = " + candidateInteger);

            if (candidateBoolean)
            {
               return candidateInteger + (10.0 - candidateX) * (10.0 - candidateX);
            }
            else
            {
               return -candidateInteger + (10.0 - candidateX) * (10.0 - candidateX);
            }
         }

      };
      
      OptimizationProblem optimizationProblem = new OptimizationProblem(listOfParametersToOptimize, sampleCostFunction);
      
      double stepChange = 0.01;
      int numberOfEvaluations = 2000;
      SimpleRandomGradientDecentParameterOptimizer optimizer = new SimpleRandomGradientDecentParameterOptimizer(stepChange, numberOfEvaluations);
      listOfParametersToOptimize = optimizer.optimize(optimizationProblem);
            
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
 
}
