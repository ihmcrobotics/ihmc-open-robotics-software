package us.ihmc.utilities.parameterOptimization;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.utilities.parameterOptimization.IndividualToEvaluate;
import us.ihmc.utilities.parameterOptimization.ListOfParametersToOptimize;
import us.ihmc.utilities.parameterOptimization.OptimizationProblem;

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

      SimpleRandomGradientDecentParameterOptimizer optimizer = new SimpleRandomGradientDecentParameterOptimizer(stepChange);

      boolean maximize = false;
      double cutoffFitness = Double.NEGATIVE_INFINITY;
      int maximumNumberOfIndividualsToEvaluate = 1000;

      OptimizationProblem optimizationProblem = new OptimizationProblem(costFunction, maximize, cutoffFitness, maximumNumberOfIndividualsToEvaluate);

      IndividualToEvaluate optimalIndividualToEvaluate = optimizer.optimize(optimizationProblem);
      ListOfParametersToOptimize optimalListOfParametersToOptimize = optimalIndividualToEvaluate.getAllParametersToOptimize();

      controller.setCurrentValues(optimalListOfParametersToOptimize);
      controller.printParameters(optimalListOfParametersToOptimize);
     
      assertTrue(controller.verifyParametersCloseToOptimal());
   }

}
