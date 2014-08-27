package us.ihmc.utilities.parameterOptimization;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

public class SimpleSimulationTest
{
   @Test
   public void testSimpleSimulation()
   {
      SimpleRobotToOptimize robot = new SimpleRobotToOptimize();
      SimpleControllerToOptimize controller = new SimpleControllerToOptimize();
      robot.setController(controller);

      SimpleSimulationIndividualToEvaluate costFunction = new SimpleSimulationIndividualToEvaluate();

      double stepChange = 0.01;
      Random random = new Random(1984L);

      SimpleRandomGradientDecentParameterOptimizer optimizer = new SimpleRandomGradientDecentParameterOptimizer(random, stepChange);

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
