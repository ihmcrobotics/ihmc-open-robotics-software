package us.ihmc.util.parameterOptimization;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class SimpleSimulationTest
{
   @Test
   public void test()
   {
      SimpleRobotToOptimize robot = new SimpleRobotToOptimize();
      SimpleControllerToOptimize controller = new SimpleControllerToOptimize();
      robot.setController(controller);

      SimpleSimulationIndividualToEvaluate costFunction = new SimpleSimulationIndividualToEvaluate();

      double stepChange = 0.01;
      int numberOfEvaluations = 1000;

      SimpleRandomGradientDecentParameterOptimizer optimizer = new SimpleRandomGradientDecentParameterOptimizer(stepChange, numberOfEvaluations);

      boolean maximize = false;
      double cutoffFitness = Double.NEGATIVE_INFINITY;
      OptimizationProblem optimizationProblem = new OptimizationProblem(costFunction, maximize, cutoffFitness);

      IndividualToEvaluate optimalIndividualToEvaluate = optimizer.optimize(optimizationProblem);
      ListOfParametersToOptimize optimalListOfParametersToOptimize = optimalIndividualToEvaluate.getAllParametersToOptimize();

      controller.setCurrentValues(optimalListOfParametersToOptimize);
      controller.printParameters(optimalListOfParametersToOptimize);
     
      assertTrue(controller.verifyParametersCloseToOptimal());
   }

}
