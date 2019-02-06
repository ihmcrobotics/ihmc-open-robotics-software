package us.ihmc.simulationConstructionSetTools.optimization;

import static us.ihmc.robotics.Assert.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.utilities.parameterOptimization.IndividualToEvaluate;
import us.ihmc.utilities.parameterOptimization.ListOfParametersToOptimize;
import us.ihmc.utilities.parameterOptimization.OptimizationProblem;
import us.ihmc.utilities.parameterOptimization.SimpleRandomGradientDecentParameterOptimizer;

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
