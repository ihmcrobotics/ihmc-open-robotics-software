package us.ihmc.utilities.parameterOptimization.generatedTestSuites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

//import us.ihmc.utilities.code.unitTesting.runner.JUnitTestSuiteRunner;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
   us.ihmc.utilities.parameterOptimization.ExampleOptimizationProblemOneTest.class,
   us.ihmc.utilities.parameterOptimization.geneticAlgorithm.GeneticAlgorithmTest.class,
   us.ihmc.utilities.parameterOptimization.geneticAlgorithm.GenotypeTest.class,
   us.ihmc.utilities.parameterOptimization.geneticAlgorithm.PopulationTest.class,
   us.ihmc.utilities.parameterOptimization.IntegerParameterToOptimizeTest.class
})

public class ParameterOptimizationDockerTestSuite
{
   public static void main(String[] args)
   {
      //new JUnitTestSuiteRunner(ParameterOptimizationDockerTestSuite.class);
   }
}

