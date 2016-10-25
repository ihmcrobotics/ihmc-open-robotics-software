package us.ihmc.utilities.parameterOptimization.generatedTestSuites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite.SuiteClasses;

import us.ihmc.tools.continuousIntegration.ContinuousIntegrationSuite;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationSuite.ContinuousIntegrationSuiteCategory;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;

/** WARNING: AUTO-GENERATED FILE. DO NOT MAKE MANUAL CHANGES TO THIS FILE. **/
@RunWith(ContinuousIntegrationSuite.class)
@ContinuousIntegrationSuiteCategory(IntegrationCategory.FAST)
@SuiteClasses
({
   us.ihmc.utilities.parameterOptimization.ExampleOptimizationProblemOneTest.class,
   us.ihmc.utilities.parameterOptimization.geneticAlgorithm.GeneticAlgorithmTest.class,
   us.ihmc.utilities.parameterOptimization.geneticAlgorithm.GenotypeTest.class,
   us.ihmc.utilities.parameterOptimization.geneticAlgorithm.PopulationTest.class,
   us.ihmc.utilities.parameterOptimization.IntegerParameterToOptimizeTest.class
})

public class ParameterOptimizationAFastTestSuite
{
   public static void main(String[] args)
   {

   }
}
